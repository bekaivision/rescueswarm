"""Фаза 4: N экземпляров mrg_slam (по одному на робота) + slam_adapter.

Не использует launch самого mrg_slam: тот добавляет python-ретранслятор облака, статический TF лидара и python
map2odom_publisher (каждый ~0.27 ядра). Здесь та же начинка (контейнер + prefiltering + scan matching + mrg_slam),
но проще:
  * вход - облако /<r>/slam_points: slam_cloud_crop убирает из /<r>/lidar/points точки пола и корпуса робота
    (иначе scan matching застревает), prefiltering переводит его в <r>/base_link по TF из URDF;
  * сырой выход scan matching (/<r>/scan_matching_odometry/raw_odom) слит с колёсной одометрией в slam_adapter
    с учётом вырожденности геометрии кадра (её считает slam_cloud_crop), оттуда его читает mrg_slam (подробности в slam_adapter.py);
  * собственный TF mrg_slam (slam_odom -> base_link) уходит в отдельный топик /<r>/slam_tf и никому не мешает:
    у <r>/base_link уже есть родитель <r>/odom (колёсная одометрия);
  * map2odom не нужен: slam_adapter читает /<r>/mrg_slam/odom2map и слитую одометрию напрямую
    и сам публикует TF world -> <r>/odom.
Привязка: init_pose каждого экземпляра = точка старта робота в world, поэтому карта mrg_slam = world.
Базовые параметры берутся из config/mrg_slam_ground.yaml, здесь только то, что зависит от робота.
Аргументы: n, poses ("x,y,yaw;..." - как в robots.launch.py), out_dir, loop_closure (false), init_guess (false), deg_lambda0 (200),
config (другой yaml), cloud_crop (true; false отключает и оценку вырожденности),
graph_exchange (auto = как loop_closure; true/false). Обмен графами mrg_slam идёт напрямую по ROS, мимо comms_sim, поэтому
без замыканий петель (когда принятые графы всё равно не нужны) он выключен: у каждого экземпляра в multi_robot_names только
собственное имя, режим обмена NONE, а общие топики /mrg_slam/odom_broadcast и /mrg_slam/slam_pose_broadcast
переназначены в закрытые топики робота (иначе роботы слышали бы друг друга по ROS в обход радио).
Список роботов (ground_1..n) считает только этот launch-файл (по n).
"""
import copy
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


# Замыкания петель (и между роботами) mrg_slam по умолчанию ВЫКЛЮЧЕНЫ: на нашем 16-лучевом лидаре и в похожих друг
# на друга комнатах они принимали ложные совмещения даже при строгом пороге (fitness 0.15: скачки позы до 10 м и 90 град,
# ATE 4.2 м против 0.6 м без них). Выключаем пустым списком кандидатов (candidate_max_xy_distance ~ 0); обмен графами
# между роботами и привязка к world по init_pose работают как обычно. Включить: loop_closure:=true.
LOOPS_OFF = {'fitness_score_thresh': 0.15, 'candidate_max_xy_distance': 0.01}
LOOPS_ON = {'fitness_score_thresh': 0.15, 'candidate_max_xy_distance': 10.0}


def launch_setup(context):
    arg = lambda k: LaunchConfiguration(k).perform(context)  # noqa: E731
    n = int(arg('n'))
    poses = [tuple(float(v) for v in p.split(',')) for p in arg('poses').split(';') if p.strip()]
    if len(poses) < n:
        raise RuntimeError(f'poses: задано {len(poses)} поз, нужно {n}')
    robots = [f'ground_{i}' for i in range(1, n + 1)]
    cfg_path = arg('config') or os.path.join(get_package_share_directory('rs_bringup'), 'config', 'mrg_slam_ground.yaml')
    crop = arg('cloud_crop').lower() == 'true'
    loops = arg('loop_closure').lower() == 'true'
    gx = arg('graph_exchange').strip().lower()
    if gx not in ('auto', 'true', 'false'):
        raise RuntimeError(f"graph_exchange: '{gx}', нужно auto, true или false")
    exchange = loops if gx == 'auto' else gx == 'true'
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)
    base = lambda k: cfg[k]['ros__parameters']  # noqa: E731

    actions = []
    for r, (x, y, yaw) in zip(robots, poses):
        shared = copy.deepcopy(base('/**'))
        shared.update({'model_namespace': r, 'use_sim_time': True, 'points_topic': f'/{r}/slam_points' if crop else f'/{r}/lidar/points',
                       'imu_topic': f'/{r}/imu'})
        pre = copy.deepcopy(base('prefiltering_component'))
        pre['base_link_frame'] = f'{r}/base_link'
        scan = copy.deepcopy(base('scan_matching_odometry_component'))
        scan['odom_frame_id'] = f'{r}/slam_odom'
        # начальное приближение из колёсной одометрии (TF <r>/odom -> <r>/base_link) по умолчанию ВЫКЛЮЧЕНО: в mrg_slam
        # lookupTransform(base@сейчас <- base@раньше) даёт обратное движение (минус шаг вместо плюс), см.
        # patches/mrg_slam-init-guess-sign.patch. Вырожденные направления всё равно закрывает slam_adapter (по колёсам),
        # а в комнатах без приближения результат тот же.
        scan['enable_robot_odometry_init_guess'] = arg('init_guess').lower() == 'true'
        scan['robot_odom_frame_id'] = f'{r}/odom'
        slam = copy.deepcopy(base('mrg_slam_component'))
        # обмен выключен: в списке только сам робот (клиентов запроса графа нет), режим NONE (запросов нет в любом случае)
        slam.update({'own_name': r, 'multi_robot_names': robots if exchange else [r],
                     'graph_exchange_mode': 'PATH_PROXIMITY' if exchange else 'NONE',
                     'map_frame_id': f'{r}/slam_map',
                     'odom_frame_id': f'{r}/slam_odom', 'x': x, 'y': y, 'yaw': yaw,
                     'init_pose': [x, y, 0.0, 0.0, 0.0, yaw]})  # x, y, z, roll, pitch, yaw
        slam.update(LOOPS_ON if loops else LOOPS_OFF)
        # общие абсолютные топики mrg_slam (позы всех экземпляров) без обмена уводим в закрытые топики робота
        no_bcast = [] if exchange else [(f'/mrg_slam/{t}', f'/{r}/mrg_slam_isolated/{t}')
                                        for t in ('odom_broadcast', 'slam_pose_broadcast')]
        # TF scan matching и mrg_slam уходит в свои топики (кадры slam_odom -> base_link никому не нужны)
        tf_remap = [('/tf', f'/{r}/slam_tf'), ('/tf_static', f'/{r}/slam_tf_static')]
        cloud = [('velodyne_points', shared['points_topic']), ('imu/data', shared['imu_topic'])]
        container = f'{r}/mrg_slam_container'
        actions.append(Node(package='rclcpp_components', executable='component_container_mt',
                            name='mrg_slam_container', namespace=r, output='screen', parameters=[shared],
                            prefix='stdbuf -oL'))  # loop closure пишет в std::cout: без этого строки теряются при остановке
        actions.append(LoadComposableNodes(target_container=container, composable_node_descriptions=[
            ComposableNode(package='mrg_slam', plugin='mrg_slam::PrefilteringComponent',
                           name='prefiltering_component', namespace=r, parameters=[pre, shared],
                           extra_arguments=[{'use_intra_process_comms': True}], remappings=cloud),
            ComposableNode(package='mrg_slam', plugin='mrg_slam::ScanMatchingOdometryComponent',
                           name='scan_matching_odometry_component', namespace=r, parameters=[scan, shared],
                           extra_arguments=[{'use_intra_process_comms': True}],
                           # сырой выход scan matching слит с колёсами в slam_adapter, оттуда его читает mrg_slam
                           remappings=tf_remap + [('scan_matching_odometry/odom', 'scan_matching_odometry/raw_odom')]),
            ComposableNode(package='mrg_slam', plugin='mrg_slam::MrgSlamComponent',
                           name='mrg_slam_component', namespace=r, parameters=[slam, shared],
                           extra_arguments=[{'use_intra_process_comms': True}],
                           remappings=[('imu/data', shared['imu_topic'])] + tf_remap + no_bcast),
        ]))
    # облако без точек пола (см. slam_cloud_crop.py), один узел на всех
    if crop:
        actions.append(Node(package='rs_mapping', executable='slam_cloud_crop', name='slam_cloud_crop', output='log',
                            parameters=[{'use_sim_time': True, 'robots': robots}]))
    actions.append(Node(package='rs_mapping', executable='slam_adapter', name='slam_adapter', output='screen',
                        parameters=[{'use_sim_time': True, 'robots': robots, 'poses': arg('poses'),
                                     'out_dir': arg('out_dir'),
                                     'deg_lambda0': float(arg('deg_lambda0'))}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('n', default_value='3'),
        DeclareLaunchArgument('poses', default_value='', description='"x,y,yaw;x,y,yaw;..." точки старта в world'),
        DeclareLaunchArgument('loop_closure', default_value='false', description='замыкания петель mrg_slam (см. комментарий выше)'),
        DeclareLaunchArgument('graph_exchange', default_value='auto', description='обмен графами mrg_slam между роботами напрямую по ROS (мимо comms_sim): auto = как loop_closure'),
        DeclareLaunchArgument('init_guess', default_value='false', description='колёсная одометрия как начальное приближение scan matching'),
        DeclareLaunchArgument('deg_lambda0', default_value='200.0', description='сумма n*n^T, при которой scan matching верим наполовину (fusion=degeneracy)'),
        DeclareLaunchArgument('config', default_value='', description='другой yaml с параметрами mrg_slam (для экспериментов)'),
        DeclareLaunchArgument('cloud_crop', default_value='true', description='убирать пол и корпус из облака (slam_cloud_crop)'),
        DeclareLaunchArgument('out_dir', default_value='', description='куда писать jumps.csv'),
        OpaqueFunction(function=launch_setup),
    ])

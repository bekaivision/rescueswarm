"""Фаза 2: автономное исследование сгенерированного мира командой роботов.

Шаги: генератор мира (seed) -> Gazebo + роботы с истинной позой -> общая карта ->
other_robots (роботы видят друг друга) -> Nav2 с collision_monitor на каждого -> explorer на каждого (аукцион) -> узел покрытия. Человек не нужен.
Аргументы: seed, size, n, headless, rviz, perception, out_dir, target_coverage, max_time (сим. с, 0 = без лимита),
localization (gt по умолчанию | slam), world_src (готовый мир вместо генерации).
Дроны (Фаза 5): drones (число, 0 = как в Фазе 4), drone_backend (light | px4), drone_alt (drone_i на drone_alt + (i-1) м),
drone_speed, air_explore (воздушные исследователи air_explorer), air_perception (тепловые детекторы дронов air_thermal_detector).
Позы старта дронов берутся из ground_truth.yaml: air_spawns. Наземные роботы по-прежнему ground_1..n в параметре robots,
дроны drone_1..m передаются узлам отдельно в air_robots.
Контракт позы: /<robot>/pose (world -> base_link, из TF узлом pose_from_tf) - единственная мировая поза для всех
потребителей; /<robot>/gt_odom - истинная поза только для оценки.
gt: TF и /odom истинные (как в Фазе 3b). slam: /odom и TF odom -> base_link от колёс (с дрейфом); TF world -> odom
публикует slam_adapter (slam.launch.py: mrg_slam на каждого робота, карта привязана к world по позам старта).
Результаты (мир, CSV, карта, график) лежат в out_dir.
Связь (Фаза 6): comms:=true - роботы обмениваются данными только пакетами через comms_sim (радиомодель),
у каждого своя карта /<r>/map и свои отметки людей, база base отдаёт общие /map и /people/*.
Аргументы: link (ideal|good|medium|poor), outages ("ground_1:60:180;drone_1:30:90"), home_on_done, comms_seed.
С comms:=false запуск ровно как в Фазе 5.
"""
import os
import shutil
import subprocess
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    arg = lambda k: LaunchConfiguration(k).perform(context)  # noqa: E731
    seed, size, n = int(arg('seed')), arg('size'), int(arg('n'))
    headless = arg('headless').lower() == 'true'
    use_rviz = arg('rviz').lower() == 'true'
    out_dir = arg('out_dir') or f'/workspace/results/phase-2/run_seed_{seed}'
    world_dir = os.path.join(out_dir, 'world')
    os.makedirs(world_dir, exist_ok=True)

    # генератор мира: один раз, до запуска симулятора
    world_src = arg('world_src')
    if world_src:
        # готовый мир (например, для опытов с изменённой геометрией): копируем вместо генерации
        shutil.copytree(world_src, world_dir, dirs_exist_ok=True)
    else:
        subprocess.run([sys.executable, '-m', 'rs_worlds.generate', '--seed', str(seed),
                        '--out', world_dir, '--size', size], check=True)
    if int(arg('drones')) > 0 and arg('drone_backend').strip().lower() == 'px4':
        # PX4 нужны в мире магнитометр, барометр, GPS и географическая привязка (мир без PX4 не меняется)
        from rs_worlds.px4_world import add_px4_systems
        wp = os.path.join(world_dir, 'world.sdf')
        with open(wp) as f:
            txt = f.read()
        with open(wp, 'w') as f:
            f.write(add_px4_systems(txt))
    with open(os.path.join(world_dir, 'ground_truth.yaml')) as f:
        gt = yaml.safe_load(f)
    poses = ';'.join(f"{p['x']},{p['y']},{p['yaw']}" for p in gt['spawn_poses'][:n])
    b = gt['building']
    region = [b['xmin'] - 1.0, b['ymin'] - 1.0, b['xmax'] + 1.0, b['ymax'] + 1.0]  # как маска покрытия
    loc = arg('localization').lower()
    if loc not in ('gt', 'slam'):
        raise RuntimeError(f"localization: '{loc}', нужно gt или slam")
    robots = [f'ground_{i}' for i in range(1, n + 1)]
    m = int(arg('drones'))
    air_robots = [f'drone_{i}' for i in range(1, m + 1)]
    air_poses = ''
    if m > 0:
        air = gt.get('air_spawns') or []
        if len(air) < m:
            raise RuntimeError(f'в мире {len(air)} стартовых поз дронов (air_spawns), нужно {m}')
        air_poses = ';'.join(f"{p['x']},{p['y']},{p['yaw']}" for p in air[:m])
    air_params = {'air_robots': air_robots} if m > 0 else {}
    comms = arg('comms').lower() == 'true'
    perception = arg('perception').lower() == 'true'
    gt_path = os.path.join(world_dir, 'ground_truth.yaml')
    if comms:
        spawn = next(z for z in gt['zones'] if z.get('type') == 'spawn')
        sx0, sy0, sx1, sy1 = spawn['rect']
        base_xyz = [(sx0 + sx1) / 2, sy0 - 0.5, 2.5]  # пост спасателей с мачтой у входа
        net_nodes = robots + air_robots + ['base']
    bringup = get_package_share_directory('rs_bringup')
    sim = {'use_sim_time': True}

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'robots.launch.py')),
            launch_arguments={
                'n': str(n), 'headless': str(headless).lower(), 'rviz': str(use_rviz).lower(),
                'world': os.path.join(world_dir, 'world.sdf'), 'localization': loc, 'poses': poses,
                # в slam world -> odom публикует slam_adapter, в gt - статический единичный
                'static_world_odom': 'false' if loc == 'slam' else 'true',
                'rviz_config': os.path.join(bringup, 'rviz', 'explore.rviz'),
                'drones': str(m), 'drone_backend': arg('drone_backend'), 'drone_alt': arg('drone_alt'),
                'drone_speed': arg('drone_speed'), 'air_poses': air_poses, 'air_explore': arg('air_explore'),
            }.items()),
        # единая мировая поза /<robot>/pose из TF (в gt это истинная поза, в slam - оценка SLAM)
        Node(package='rs_mapping', executable='pose_from_tf', name='pose_from_tf', output='screen',
             parameters=[sim, {'robots': robots}]),
        Node(package='rs_mapping', executable='shared_map', name='shared_map', output='screen',
             parameters=[sim, {'robots': robots, 'gt_file': os.path.join(world_dir, 'ground_truth.yaml'), **air_params,
                               **({'layers': True} if comms else {})}]),
        # роботы видят друг друга: виртуальные сканы для costmap и collision_monitor
        Node(package='rs_mapping', executable='other_robots', name='other_robots', output='screen',
             parameters=[sim, {'robots': robots}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'nav2_multi.launch.py')),
            launch_arguments={'n': str(n), **({'map_topic': '/{ns}/map'} if comms else {})}.items()),
    ]
    if comms:
        actions.append(Node(
            package='rs_comms', executable='comms_sim', name='comms_sim', output='screen',
            parameters=[sim, {'nodes': net_nodes, 'robots': robots, **air_params, 'gt_file': gt_path,
                              'base_xyz': base_xyz, 'level': arg('link'), 'outages': arg('outages'),
                              'out_dir': out_dir, 'seed': int(arg('comms_seed'))}]))
        for node in net_nodes:
            kind = 'base' if node == 'base' else ('air' if node in air_robots else 'ground')
            p = {'me': node, 'kind': kind, 'robots': robots, **air_params, 'gt_file': gt_path, 'log_dir': out_dir}
            if perception and kind != 'air':
                p.update({'fusion': True, 'obs_topic': f'/{node}/comms/obs'})
                if kind == 'ground':
                    p.update({'map_topic': f'/{node}/map', 'people_ns': f'/{node}/people', 'out_dir': ''})
                else:
                    p.update({'map_topic': '/map', 'people_ns': '/people', 'out_dir': out_dir})
            else:
                p['fusion'] = False
            # без name=: в процессе два узла (агент связи и слияние), name переименовал бы оба
            actions.append(Node(package='rs_comms', executable='onboard', namespace=node,
                                output='screen', parameters=[sim, p]))
    if loc == 'slam':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'slam.launch.py')),
            launch_arguments={'n': str(n), 'poses': poses, 'out_dir': out_dir}.items()))
    for i, r in enumerate(robots, start=1):
        ex_params = {'robot_id': r, 'robots': robots, 'region': region, 'publish_cells': i == 1}
        ex_remap = []
        if comms:
            sp = gt['spawn_poses'][i - 1]
            ex_params.update({'global_keys': True, 'home_on_done': arg('home_on_done').lower() == 'true',
                              'home': [float(sp['x']), float(sp['y']), float(sp['yaw'])]})
            ex_remap = [('/map', f'/{r}/map'), ('/fleet/poses', f'/{r}/fleet/poses'),
                        ('/exploration/bids', f'/{r}/exploration/bids'),
                        ('/people/inspect_requests', f'/{r}/people/inspect_requests')]
        actions.append(Node(
            package='rs_exploration', executable='explorer', name='explorer', namespace=r,
            output='screen', parameters=[sim, ex_params], **({'remappings': ex_remap} if comms else {})))
    if m > 0 and arg('air_explore').lower() == 'true':
        for d in air_robots:
            actions.append(Node(
                package='rs_exploration', executable='air_explorer', name='air_explorer', namespace=d,
                output='screen',
                parameters=[sim, {'robot': d, 'air_robots': air_robots, 'region': region}],
                **({'remappings': [('/map_air', f'/{d}/map_air'),
                                   ('/exploration/air_bids', f'/{d}/exploration/air_bids')]} if comms else {})))
    cov = Node(package='rs_eval', executable='coverage', name='coverage', output='screen',
               parameters=[sim, {'mask_dir': world_dir, 'robots': robots, 'out_dir': out_dir,
                                 'target': float(arg('target_coverage')),
                                 'search_wait': arg('perception').lower() == 'true',
                                 'max_time': float(arg('max_time')), **air_params,
                                 **({'comms': True} if comms else {})}])
    actions.append(cov)
    if arg('perception').lower() == 'true':
        for r in robots:
            actions.append(Node(
                package='rs_perception', executable='thermal_detector', name='thermal_detector',
                namespace=r, output='screen',
                parameters=[sim, {'robot': r, 'debug_dir': os.path.join(out_dir, 'debug')}],
                **({'remappings': [('/perception/detections', f'/{r}/perception/detections')]} if comms else {})))
        if arg('rgb').lower() == 'true':
            actions.append(Node(package='rs_perception', executable='rgb_detector', name='rgb_detector',
                                output='screen', parameters=[sim, {'robots': robots}]))
        if m > 0 and arg('air_perception').lower() == 'true':
            for d in air_robots:
                actions.append(Node(
                    package='rs_perception', executable='air_thermal_detector', name='air_thermal_detector',
                    namespace=d, output='screen', parameters=[sim, {'robot': d}],
                    **({'remappings': [('/perception/detections', f'/{d}/perception/detections')]} if comms else {})))
        # robots обязателен: порядок имён должен совпадать с /fleet/poses (раньше брался список по умолчанию из 3 роботов)
        if not comms:  # в режиме связи слияние делает onboard на каждом узле (на базе - общее)
            actions.append(Node(package='rs_perception', executable='people_fusion', name='people_fusion',
                                output='screen', parameters=[sim, {'out_dir': out_dir, 'robots': robots, **air_params}]))
    # покрытие достигнуто / время вышло: гасим весь запуск
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=cov, on_exit=[EmitEvent(event=Shutdown(reason='exploration finished'))])))
    return actions


def generate_launch_description():
    decl = [
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument('size', default_value='medium'),
        DeclareLaunchArgument('n', default_value='3'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('out_dir', default_value='', description='по умолчанию /workspace/results/phase-2/run_seed_<seed>'),
        DeclareLaunchArgument('target_coverage', default_value='0.95'),
        DeclareLaunchArgument('perception', default_value='true', description='тепловые детекторы и слияние отметок людей'),
        DeclareLaunchArgument('rgb', default_value='true', description='YOLO по RGB как дополнительный признак (нужен perception)'),
        DeclareLaunchArgument('world_src', default_value='', description='каталог готового мира (world.sdf, ground_truth.yaml, ...) вместо генерации по seed'),
        DeclareLaunchArgument('localization', default_value='gt', description='gt: истинная поза; slam: одометрия колёс + TF world->odom от slam_adapter (mrg_slam)'),
        DeclareLaunchArgument('max_time', default_value='0', description='сим. секунды, 0 = без лимита'),
        DeclareLaunchArgument('drones', default_value='0', description='число дронов drone_1..drone_M'),
        DeclareLaunchArgument('drone_backend', default_value='light', description='light или px4'),
        DeclareLaunchArgument('drone_alt', default_value='4.5', description='высота drone_1, м; drone_i на (i-1) м выше'),
        DeclareLaunchArgument('drone_speed', default_value='2.0', description='крейсерская скорость дрона, м/с'),
        DeclareLaunchArgument('air_explore', default_value='true', description='запускать air_explorer на каждом дроне'),
        DeclareLaunchArgument('comms', default_value='false', description='режим связи Фазы 6: обмен только через comms_sim'),
        DeclareLaunchArgument('link', default_value='good', description='уровень связи: ideal|good|medium|poor'),
        DeclareLaunchArgument('outages', default_value='', description='отказы узлов "ground_1:60:180;drone_1:30:90" (узел:с:до, сим. секунды)'),
        DeclareLaunchArgument('home_on_done', default_value='true', description='в режиме связи: возвращаться на базу после исследования'),
        DeclareLaunchArgument('comms_seed', default_value='1', description='зерно потерь пакетов в comms_sim'),
        DeclareLaunchArgument('air_perception', default_value='true', description='запускать air_thermal_detector на каждом дроне (нужен perception)'),
    ]
    return LaunchDescription(decl + [OpaqueFunction(function=launch_setup)])

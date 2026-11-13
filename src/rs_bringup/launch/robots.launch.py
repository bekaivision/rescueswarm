"""Фаза 1: мир Gazebo + N наземных роботов ground_1..ground_N.

У каждого робота: свой robot_state_publisher (namespace + frame_prefix), свой
ros_gz_bridge, статический TF world -> ground_i/odom по позе старта.
Аргументы: n, headless, rviz, rviz_config, world, spacing, poses, gt_pose, localization, static_world_odom,
а также частоты и разрешения сенсоров.
Дроны (Фаза 5): drones (число, 0 по умолчанию: тогда запуск как раньше), drone_backend (light | px4), drone_alt
(drone_i летит на drone_alt + (i-1) м, дроны не делят высоту), drone_speed, air_poses ("x,y,yaw;..." как poses).
Для каждого дрона drone_i: robot_state_publisher (статические TF сенсоров), создание модели, мост
(cmd_vel, gt_odom, depth/points, thermal), узел rs_drone/drone_adapter. air_explore здесь только принимается
(его использует explore.launch.py).
localization: '' (по умолчанию, как раньше: решает gt_pose), 'gt' (= gt_pose true: /odom и TF истинные,
статический world -> odom единичный), 'slam' (/odom и TF odom -> base_link от колёс; статический
world -> odom по умолчанию НЕ запускается: его публикует SLAM-адаптер).
static_world_odom: '' (авто: запускать, кроме localization=slam), 'true' / 'false' - принудительно.
Истинная поза в /<robot>/gt_odom есть всегда (без TF).
"""
import os

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SENSOR_ARGS = {
    'lidar_rate': '10', 'lidar_samples': '360', 'lidar_beams': '16',
    'rgb_rate': '5', 'rgb_width': '320', 'rgb_height': '240',
    'thermal_rate': '5', 'thermal_width': '160', 'thermal_height': '120',
    'imu_rate': '50', 'imu_noise': 'true',
}


def bridge_args(ns):
    # [ : gz -> ROS, ] : ROS -> gz
    return [
        f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        f'/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        f'/{ns}/gt_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        f'/{ns}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        f'/{ns}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        f'/{ns}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        f'/{ns}/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        f'/{ns}/rgb/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        f'/{ns}/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        f'/{ns}/thermal/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        f'/{ns}/thermal/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
    ]


def drone_bridge_args(ns):
    # [ : gz -> ROS, ] : ROS -> gz
    return [
        f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        f'/{ns}/gt_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        f'/{ns}/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        f'/{ns}/thermal/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        f'/{ns}/thermal/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
    ]


# PX4: высота base_link X500 над землёй в покое (поза x500_base 0 0 .24 при модели на z = 0)
PX4_BASE_Z = 0.24
PX4_AGENT_PORT = '8888'


def px4_drone_sdf(ns):
    """Модель дрона для PX4: X500 из PX4 (merge: те же base_link и сенсоры IMU, магнитометр, барометр,
    GPS, моторы на /<ns>/command/motor_speed) плюс наши надирные сенсоры и истинная поза.
    Сенсоры и их кадры те же, что в rs_description/urdf/drone_sensors.xacro (держать одинаковыми):
    звенья depth_link и thermal_link на 0.08 м ниже base_link, ось x вниз."""
    def nadir(name, sensor):
        return f'''
    <link name="{name}">
      <pose relative_to="base_link">0 0 -0.08 0 1.5707963 0</pose>
      <inertial><mass>0.001</mass><inertia><ixx>1e-7</ixx><iyy>1e-7</iyy><izz>1e-7</izz></inertia></inertial>
      {sensor}
    </link>
    <joint name="{name}_joint" type="fixed"><parent>base_link</parent><child>{name}</child></joint>'''
    depth = f'''<sensor name="depth" type="depth_camera">
        <always_on>1</always_on><update_rate>5</update_rate><topic>/{ns}/depth</topic>
        <gz_frame_id>{ns}/depth_link</gz_frame_id>
        <camera><horizontal_fov>1.396</horizontal_fov>
          <image><width>160</width><height>120</height><format>R_FLOAT32</format></image>
          <clip><near>0.3</near><far>15</far></clip></camera>
      </sensor>'''
    thermal = f'''<sensor name="thermal" type="thermal">
        <always_on>1</always_on><update_rate>5</update_rate><topic>/{ns}/thermal/image_raw</topic>
        <gz_frame_id>{ns}/thermal_link</gz_frame_id>
        <camera><horizontal_fov>1.396</horizontal_fov>
          <image><width>160</width><height>120</height><format>L16</format></image>
          <clip><near>0.1</near><far>50</far></clip></camera>
      </sensor>'''
    return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{ns}">
    <include merge="true"><uri>model://x500</uri></include>{nadir('depth_link', depth)}{nadir('thermal_link', thermal)}
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>{ns}/base_link</robot_base_frame>
      <odom_topic>/{ns}/gt_odom</odom_topic>
      <tf_topic>/{ns}/gt_tf</tf_topic>
      <odom_publish_frequency>30</odom_publish_frequency>
      <dimensions>3</dimensions>
    </plugin>
  </model>
</sdf>'''


def parse_poses(text):
    # "x,y,yaw;x,y,yaw;..."
    return [tuple(float(v) for v in p.split(',')) for p in text.split(';') if p.strip()]


def world_name(path):
    """Имя мира из атрибута <world name="..."> файла SDF."""
    import re
    with open(path) as f:
        m = re.search(r'<world\s+name=["\']([^"\']+)', f.read())
    return m.group(1) if m else 'default'


def launch_setup(context):
    n = int(LaunchConfiguration('n').perform(context))
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    use_rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    spacing = float(LaunchConfiguration('spacing').perform(context))
    world = LaunchConfiguration('world').perform(context)
    gt_pose = LaunchConfiguration('gt_pose').perform(context).lower() == 'true'
    loc = LaunchConfiguration('localization').perform(context).strip().lower()
    if loc not in ('', 'gt', 'slam'):
        raise RuntimeError(f"localization: '{loc}', нужно gt или slam")
    if loc == 'gt':
        gt_pose = True
    elif loc == 'slam':
        gt_pose = False
    swo = LaunchConfiguration('static_world_odom').perform(context).strip().lower()
    static_world_odom = (swo == 'true') if swo else loc != 'slam'
    poses_arg = LaunchConfiguration('poses').perform(context).strip()
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    if not rviz_config:
        rviz_config = os.path.join(get_package_share_directory('rs_bringup'), 'rviz', 'robots.rviz')
    poses = []
    if poses_arg:
        # "x,y,yaw;x,y,yaw;..."
        poses = [tuple(float(v) for v in p.split(',')) for p in poses_arg.split(';') if p.strip()]
        if len(poses) < n:
            raise RuntimeError(f'poses: задано {len(poses)} поз, нужно {n}')
    m = int(LaunchConfiguration('drones').perform(context))
    drone_backend = LaunchConfiguration('drone_backend').perform(context).strip().lower()
    if drone_backend not in ('light', 'px4'):
        raise RuntimeError(f"drone_backend: '{drone_backend}', нужно light или px4")
    drone_alt = float(LaunchConfiguration('drone_alt').perform(context))
    drone_speed = float(LaunchConfiguration('drone_speed').perform(context))
    air_poses = parse_poses(LaunchConfiguration('air_poses').perform(context))
    if m > 0 and air_poses and len(air_poses) < m:
        raise RuntimeError(f'air_poses: задано {len(air_poses)} поз, нужно {m}')
    bringup = get_package_share_directory('rs_bringup')
    desc = get_package_share_directory('rs_description')
    if not world:
        world = os.path.join(bringup, 'worlds', 'phase1.sdf')
    xacro_file = os.path.join(desc, 'urdf', 'ground_robot.urdf.xacro')
    sim_time = {'use_sim_time': True}

    gz_cmd = ['gz', 'sim', '-r']
    if headless:
        gz_cmd += ['-s', '--headless-rendering']
    px4 = m > 0 and drone_backend == 'px4'
    gz_env = {}
    if px4:
        # модели PX4 (x500 и её части) для <include> в модели дрона
        px4_models = os.environ.get('PX4_GZ_MODELS', '/opt/px4/Tools/simulation/gz/models')
        gz_env['GZ_SIM_RESOURCE_PATH'] = ':'.join(
            p for p in (os.environ.get('GZ_SIM_RESOURCE_PATH', ''), px4_models) if p)
    actions = [ExecuteProcess(cmd=gz_cmd + [world], output='screen', additional_env=gz_env)]
    if px4:
        # один агент XRCE-DDS на все экземпляры PX4, раньше них; свой Fast DDS внутри (обёртка в образе)
        actions.append(ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', PX4_AGENT_PORT], output='log'))

    # /clock один раз на всех
    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'))

    mappings = {k: LaunchConfiguration(k).perform(context) for k in SENSOR_ARGS}
    if gt_pose:
        mappings['gt_pose'] = 'true'
    for i in range(1, n + 1):
        ns = f'ground_{i}'
        if poses:
            x, y, yaw = poses[i - 1]
        else:
            x, y, yaw = 0.0, (i - (n + 1) / 2.0) * spacing, 0.0
        # gt_pose: odom уже в мировой системе, значит world -> odom единичный
        tx, ty, tyaw = (0.0, 0.0, 0.0) if gt_pose else (x, y, yaw)
        urdf = xacro.process_file(xacro_file, mappings={'robot_name': ns, **mappings}).toxml()

        actions.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=ns, output='screen',
            parameters=[{'robot_description': urdf, 'frame_prefix': f'{ns}/'}, sim_time],
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static')]))
        actions.append(Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=['-name', ns, '-string', urdf, '-x', str(x), '-y', str(y),
                       '-z', '0.12', '-Y', str(yaw)]))
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name=f'{ns}_bridge',
            arguments=bridge_args(ns), output='screen', parameters=[sim_time],
            remappings=[(f'/{ns}/tf', '/tf')]))
        if static_world_odom:
            actions.append(Node(
                package='tf2_ros', executable='static_transform_publisher',
                name=f'{ns}_world_tf', output='log', parameters=[sim_time],
                arguments=['--x', str(tx), '--y', str(ty), '--z', '0', '--yaw', str(tyaw),
                           '--frame-id', 'world', '--child-frame-id', f'{ns}/odom']))

    drone_xacro = os.path.join(desc, 'urdf', 'drone_light.urdf.xacro')
    for i in range(1, m + 1):
        ns = f'drone_{i}'
        if air_poses:
            x, y, yaw = air_poses[i - 1]
        else:
            x, y, yaw = (i - (m + 1) / 2.0) * spacing, 0.0, 0.0
        # URDF лёгкого дрона нужен и для PX4: robot_state_publisher берёт из него статические TF сенсоров
        durdf = xacro.process_file(drone_xacro, mappings={'robot_name': ns}).toxml()
        actions.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=ns, output='screen',
            parameters=[{'robot_description': durdf, 'frame_prefix': f'{ns}/'}, sim_time],
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static')]))
        params = {'robot': ns, 'backend': drone_backend, 'altitude': drone_alt + (i - 1) * 1.0, 'speed': drone_speed}
        if px4:
            actions.append(Node(
                package='ros_gz_sim', executable='create', output='screen',
                arguments=['-name', ns, '-string', px4_drone_sdf(ns), '-x', str(x), '-y', str(y),
                           '-z', '0.0', '-Y', str(yaw)]))
            actions.append(Node(
                package='ros_gz_bridge', executable='parameter_bridge', name=f'{ns}_bridge',
                arguments=drone_bridge_args(ns)[1:], output='screen', parameters=[sim_time]))
            # экземпляр PX4 i подключается к модели drone_i; MAV_SYS_ID = i + 1
            actions.append(ExecuteProcess(
                cmd=[os.path.join(get_package_prefix('rs_drone'), 'lib', 'rs_drone', 'px4_instance.sh'),
                     str(i), ns, world_name(world)], output='screen'))
            params.update(origin=[x, y, PX4_BASE_Z], yaw=yaw, target_system=i + 1, land_z=PX4_BASE_Z)
        else:
            actions.append(Node(
                package='ros_gz_sim', executable='create', output='screen',
                arguments=['-name', ns, '-string', durdf, '-x', str(x), '-y', str(y),
                           '-z', '0.14', '-Y', str(yaw)]))
            actions.append(Node(
                package='ros_gz_bridge', executable='parameter_bridge', name=f'{ns}_bridge',
                arguments=drone_bridge_args(ns), output='screen', parameters=[sim_time]))
        actions.append(Node(
            package='rs_drone', executable='drone_adapter', name='drone_adapter', namespace=ns,
            output='screen', parameters=[sim_time, params]))

    if use_rviz:
        actions.append(Node(
            package='rviz2', executable='rviz2', output='log', parameters=[sim_time],
            arguments=['-d', rviz_config]))
    return actions


def generate_launch_description():
    decl = [
        DeclareLaunchArgument('n', default_value='3', description='число роботов'),
        DeclareLaunchArgument('headless', default_value='false', description='gz без окна'),
        DeclareLaunchArgument('rviz', default_value='true', description='запустить RViz'),
        DeclareLaunchArgument('world', default_value='', description='путь к SDF (по умолчанию phase1.sdf)'),
        DeclareLaunchArgument('spacing', default_value='2.0', description='расстояние между роботами по y, м'),
        DeclareLaunchArgument('gt_pose', default_value='false', description='истинная поза из Gazebo вместо одометрии колёс'),
        DeclareLaunchArgument('localization', default_value='', description="'' (как раньше, по gt_pose), 'gt' или 'slam'"),
        DeclareLaunchArgument('static_world_odom', default_value='', description="'' (авто), 'true' или 'false': статический TF world -> odom"),
        DeclareLaunchArgument('poses', default_value='', description='"x,y,yaw;x,y,yaw;..." (пусто: ряд по y с шагом spacing)'),
        DeclareLaunchArgument('rviz_config', default_value='', description='путь к .rviz (по умолчанию robots.rviz)'),
        DeclareLaunchArgument('drones', default_value='0', description='число дронов drone_1..drone_M'),
        DeclareLaunchArgument('drone_backend', default_value='light', description='light (Gazebo velocity-control) или px4'),
        DeclareLaunchArgument('drone_alt', default_value='4.5', description='высота drone_1, м; drone_i на (i-1) м выше'),
        DeclareLaunchArgument('drone_speed', default_value='2.0', description='крейсерская скорость дрона, м/с'),
        DeclareLaunchArgument('air_poses', default_value='', description='"x,y,yaw;..." старт дронов (пусто: ряд по x в начале координат)'),
        DeclareLaunchArgument('air_explore', default_value='true', description='здесь не используется (для explore.launch.py)'),
    ]
    decl += [DeclareLaunchArgument(k, default_value=v) for k, v in SENSOR_ARGS.items()]
    return LaunchDescription(decl + [OpaqueFunction(function=launch_setup)])

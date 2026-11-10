"""Фаза 1: мир Gazebo + N наземных роботов ground_1..ground_N.

У каждого робота: свой robot_state_publisher (namespace + frame_prefix), свой
ros_gz_bridge, статический TF world -> ground_i/odom по позе старта.
Аргументы: n, headless, rviz, rviz_config, world, spacing, poses, gt_pose, localization, static_world_odom,
а также частоты и разрешения сенсоров.
localization: '' (по умолчанию, как раньше: решает gt_pose), 'gt' (= gt_pose true: /odom и TF истинные,
статический world -> odom единичный), 'slam' (/odom и TF odom -> base_link от колёс; статический
world -> odom по умолчанию НЕ запускается: его публикует SLAM-адаптер).
static_world_odom: '' (авто: запускать, кроме localization=slam), 'true' / 'false' - принудительно.
Истинная поза в /<robot>/gt_odom есть всегда (без TF).
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
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
    bringup = get_package_share_directory('rs_bringup')
    desc = get_package_share_directory('rs_description')
    if not world:
        world = os.path.join(bringup, 'worlds', 'phase1.sdf')
    xacro_file = os.path.join(desc, 'urdf', 'ground_robot.urdf.xacro')
    sim_time = {'use_sim_time': True}

    gz_cmd = ['gz', 'sim', '-r']
    if headless:
        gz_cmd += ['-s', '--headless-rendering']
    actions = [ExecuteProcess(cmd=gz_cmd + [world], output='screen')]

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
    ]
    decl += [DeclareLaunchArgument(k, default_value=v) for k, v in SENSOR_ARGS.items()]
    return LaunchDescription(decl + [OpaqueFunction(function=launch_setup)])

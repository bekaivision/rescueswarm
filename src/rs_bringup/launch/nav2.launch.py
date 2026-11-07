"""Nav2 одного наземного робота в namespace <namespace> (глобальный кадр world).

Запускаются planner, controller, behaviors, bt_navigator, collision_monitor и lifecycle_manager.
TF общий (/tf, /tf_static): в namespace его НЕ переносим.
controller и behaviors пишут в /<namespace>/cmd_vel_nav (geometry_msgs/Twist), collision_monitor
пропускает команду в /<namespace>/cmd_vel (его читает мост gz) и останавливает робота рядом с другим.
Аргументы: namespace, params_file, use_sim_time, map_topic, autostart, log_level.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml

LIFECYCLE_NODES = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'collision_monitor']


def generate_launch_description():
    bringup = get_package_share_directory('rs_bringup')
    ns = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    # сначала подставляем <robot_namespace> и <map_topic>, потом добавляем корневой ключ namespace
    replaced = ReplaceString(
        source_file=LaunchConfiguration('params_file'),
        replacements={'<robot_namespace>': ns, '<map_topic>': LaunchConfiguration('map_topic')})
    params = ParameterFile(
        RewrittenYaml(source_file=replaced, root_key=ns, param_rewrites={}, convert_types=True),
        allow_substs=True)

    def nav_node(pkg, exe, remap_cmd=False):
        # команды скорости идут через collision_monitor
        return Node(package=pkg, executable=exe, name=exe, output='screen',
                    parameters=[params], arguments=['--ros-args', '--log-level', log_level],
                    remappings=[('cmd_vel', 'cmd_vel_nav')] if remap_cmd else [])

    group = GroupAction([
        PushRosNamespace(ns),
        SetParameter('use_sim_time', use_sim_time),
        nav_node('nav2_controller', 'controller_server', remap_cmd=True),
        nav_node('nav2_planner', 'planner_server'),
        nav_node('nav2_behaviors', 'behavior_server', remap_cmd=True),
        nav_node('nav2_bt_navigator', 'bt_navigator'),
        nav_node('nav2_collision_monitor', 'collision_monitor'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'autostart': autostart, 'node_names': LIFECYCLE_NODES,
                          'bond_timeout': 10.0}],
             arguments=['--ros-args', '--log-level', log_level]),
    ])
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='ground_1'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(bringup, 'config', 'nav2_ground.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        group,
    ])

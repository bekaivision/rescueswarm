"""Nav2 для ground_1..ground_N одной командой (обёртка над nav2.launch.py)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    n = int(LaunchConfiguration('n').perform(context))
    map_topic = LaunchConfiguration('map_topic').perform(context)
    path = os.path.join(get_package_share_directory('rs_bringup'), 'launch', 'nav2.launch.py')
    # новый источник на каждого робота: иначе описание кэшируется и действия запускаются дважды
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments={
        'namespace': f'ground_{i}',
        # '{ns}' в теме карты заменяется именем робота (режим связи: у каждого своя карта /<ns>/map)
        'map_topic': map_topic.format(ns=f'ground_{i}') if '{ns}' in map_topic else map_topic,
        'params_file': LaunchConfiguration('params_file').perform(context),
    }.items()) for i in range(1, n + 1)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('n', default_value='3'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            get_package_share_directory('rs_bringup'), 'config', 'nav2_ground.yaml')),
        OpaqueFunction(function=launch_setup),
    ])

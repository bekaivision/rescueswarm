"""Фаза 4, проверка: один робот ground_1 + один экземпляр mrg_slam, робот едет по кругу.

Состав: robots.launch.py (n=1, gt_pose, без окон) -> ретранслятор облака (смена кадра) -> mrg_slam
(external/mrg_slam, конфиг config/mrg_slam_ground.yaml) -> узел slam_smoke_tools.py run (движение
и сравнение позы SLAM с истинной). Когда он закончит, всё останавливается.
Аргументы: duration (сим. с движения), out_dir, headless, world.
Результаты: pose_error.csv, resources.csv, summary.json в out_dir.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory('rs_bringup')
    mrg = get_package_share_directory('mrg_slam')
    tools = os.path.join(bringup, 'launch', 'slam_smoke_tools.py')
    robot = 'ground_1'

    robots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'robots.launch.py')),
        launch_arguments={'n': '1', 'headless': LaunchConfiguration('headless'), 'rviz': 'false',
                          'gt_pose': 'true', 'world': LaunchConfiguration('world')}.items())
    relay = ExecuteProcess(cmd=['python3', tools, 'relay', robot], output='log')
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mrg, 'launch', 'mrg_slam.launch.py')),
        launch_arguments={'config': os.path.join(bringup, 'config', 'mrg_slam_ground.yaml'),
                          'model_namespace': robot, 'use_sim_time': 'true'}.items())
    runner = ExecuteProcess(
        cmd=['python3', tools, 'run', robot, LaunchConfiguration('duration'), LaunchConfiguration('out_dir')],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('duration', default_value='120', description='сим. секунд движения'),
        DeclareLaunchArgument('out_dir', default_value='/workspace/results/phase-4/smoke'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('world', default_value='', description='SDF мира (пусто: phase1.sdf)'),
        robots,
        TimerAction(period=8.0, actions=[relay, slam]),
        TimerAction(period=10.0, actions=[runner]),
        RegisterEventHandler(OnProcessExit(target_action=runner, on_exit=[EmitEvent(event=Shutdown())])),
    ])

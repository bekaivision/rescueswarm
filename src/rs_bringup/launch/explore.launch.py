"""Фаза 2: автономное исследование сгенерированного мира командой роботов.

Шаги: генератор мира (seed) -> Gazebo + роботы с истинной позой -> общая карта ->
other_robots (роботы видят друг друга) -> Nav2 с collision_monitor на каждого -> explorer на каждого (аукцион) -> узел покрытия. Человек не нужен.
Аргументы: seed, size, n, headless, rviz, perception, out_dir, target_coverage, max_time (сим. с, 0 = без лимита).
Результаты (мир, CSV, карта, график) лежат в out_dir.
"""
import os
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
    subprocess.run([sys.executable, '-m', 'rs_worlds.generate', '--seed', str(seed),
                    '--out', world_dir, '--size', size], check=True)
    with open(os.path.join(world_dir, 'ground_truth.yaml')) as f:
        gt = yaml.safe_load(f)
    poses = ';'.join(f"{p['x']},{p['y']},{p['yaw']}" for p in gt['spawn_poses'][:n])
    b = gt['building']
    region = [b['xmin'] - 1.0, b['ymin'] - 1.0, b['xmax'] + 1.0, b['ymax'] + 1.0]  # как маска покрытия
    robots = [f'ground_{i}' for i in range(1, n + 1)]
    bringup = get_package_share_directory('rs_bringup')
    sim = {'use_sim_time': True}

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'robots.launch.py')),
            launch_arguments={
                'n': str(n), 'headless': str(headless).lower(), 'rviz': str(use_rviz).lower(),
                'world': os.path.join(world_dir, 'world.sdf'), 'gt_pose': 'true', 'poses': poses,
                'rviz_config': os.path.join(bringup, 'rviz', 'explore.rviz'),
            }.items()),
        Node(package='rs_mapping', executable='shared_map', name='shared_map', output='screen',
             parameters=[sim, {'robots': robots, 'gt_file': os.path.join(world_dir, 'ground_truth.yaml')}]),
        # роботы видят друг друга: виртуальные сканы для costmap и collision_monitor
        Node(package='rs_mapping', executable='other_robots', name='other_robots', output='screen',
             parameters=[sim, {'robots': robots}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'nav2_multi.launch.py')),
            launch_arguments={'n': str(n)}.items()),
    ]
    for i, r in enumerate(robots, start=1):
        actions.append(Node(
            package='rs_exploration', executable='explorer', name='explorer', namespace=r,
            output='screen',
            parameters=[sim, {'robot_id': r, 'robots': robots, 'region': region, 'publish_cells': i == 1}]))
    cov = Node(package='rs_eval', executable='coverage', name='coverage', output='screen',
               parameters=[sim, {'mask_dir': world_dir, 'robots': robots, 'out_dir': out_dir,
                                 'target': float(arg('target_coverage')),
                                 'search_wait': arg('perception').lower() == 'true',
                                 'max_time': float(arg('max_time'))}])
    actions.append(cov)
    if arg('perception').lower() == 'true':
        for r in robots:
            actions.append(Node(
                package='rs_perception', executable='thermal_detector', name='thermal_detector',
                namespace=r, output='screen',
                parameters=[sim, {'robot': r, 'debug_dir': os.path.join(out_dir, 'debug')}]))
        actions.append(Node(package='rs_perception', executable='people_fusion', name='people_fusion',
                            output='screen', parameters=[sim, {'out_dir': out_dir}]))
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
        DeclareLaunchArgument('max_time', default_value='0', description='сим. секунды, 0 = без лимита'),
    ]
    return LaunchDescription(decl + [OpaqueFunction(function=launch_setup)])

"""Фаза 2: автономное исследование сгенерированного мира командой роботов.

Шаги: генератор мира (seed) -> Gazebo + роботы с истинной позой -> общая карта ->
other_robots (роботы видят друг друга) -> Nav2 с collision_monitor на каждого -> explorer на каждого (аукцион) -> узел покрытия. Человек не нужен.
Аргументы: seed, size, n, headless, rviz, perception, out_dir, target_coverage, max_time (сим. с, 0 = без лимита),
localization (gt по умолчанию | slam), world_src (готовый мир вместо генерации).
Контракт позы: /<robot>/pose (world -> base_link, из TF узлом pose_from_tf) - единственная мировая поза для всех
потребителей; /<robot>/gt_odom - истинная поза только для оценки.
gt: TF и /odom истинные (как в Фазе 3b). slam: /odom и TF odom -> base_link от колёс (с дрейфом); TF world -> odom
публикует slam_adapter (slam.launch.py: mrg_slam на каждого робота, карта привязана к world по позам старта).
Результаты (мир, CSV, карта, график) лежат в out_dir.
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
    with open(os.path.join(world_dir, 'ground_truth.yaml')) as f:
        gt = yaml.safe_load(f)
    poses = ';'.join(f"{p['x']},{p['y']},{p['yaw']}" for p in gt['spawn_poses'][:n])
    b = gt['building']
    region = [b['xmin'] - 1.0, b['ymin'] - 1.0, b['xmax'] + 1.0, b['ymax'] + 1.0]  # как маска покрытия
    loc = arg('localization').lower()
    if loc not in ('gt', 'slam'):
        raise RuntimeError(f"localization: '{loc}', нужно gt или slam")
    robots = [f'ground_{i}' for i in range(1, n + 1)]
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
            }.items()),
        # единая мировая поза /<robot>/pose из TF (в gt это истинная поза, в slam - оценка SLAM)
        Node(package='rs_mapping', executable='pose_from_tf', name='pose_from_tf', output='screen',
             parameters=[sim, {'robots': robots}]),
        Node(package='rs_mapping', executable='shared_map', name='shared_map', output='screen',
             parameters=[sim, {'robots': robots, 'gt_file': os.path.join(world_dir, 'ground_truth.yaml')}]),
        # роботы видят друг друга: виртуальные сканы для costmap и collision_monitor
        Node(package='rs_mapping', executable='other_robots', name='other_robots', output='screen',
             parameters=[sim, {'robots': robots}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'nav2_multi.launch.py')),
            launch_arguments={'n': str(n)}.items()),
    ]
    if loc == 'slam':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'slam.launch.py')),
            launch_arguments={'n': str(n), 'poses': poses, 'out_dir': out_dir}.items()))
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
        if arg('rgb').lower() == 'true':
            actions.append(Node(package='rs_perception', executable='rgb_detector', name='rgb_detector',
                                output='screen', parameters=[sim, {'robots': robots}]))
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
        DeclareLaunchArgument('rgb', default_value='true', description='YOLO по RGB как дополнительный признак (нужен perception)'),
        DeclareLaunchArgument('world_src', default_value='', description='каталог готового мира (world.sdf, ground_truth.yaml, ...) вместо генерации по seed'),
        DeclareLaunchArgument('localization', default_value='gt', description='gt: истинная поза; slam: одометрия колёс + TF world->odom от slam_adapter (mrg_slam)'),
        DeclareLaunchArgument('max_time', default_value='0', description='сим. секунды, 0 = без лимита'),
    ]
    return LaunchDescription(decl + [OpaqueFunction(function=launch_setup)])

#!/usr/bin/env python3
"""Только для разработки: проверка аварийной остановки (collision_monitor) без планировщика.

  collision_test.py SCENARIO [--time 12]
Сценарии (роботы стоят по позам из launch, команды идут в /ground_i/cmd_vel_nav, как у controller):
  head_static  - ground_1 едет вперёд на стоящий ground_2 (лицом друг к другу)
  head_both    - оба едут навстречу друг другу
  from_behind  - ground_1 едет вперёд в корму стоящего ground_2
  reverse_into - ground_1 едет назад в стоящий за ним ground_2
  side         - ground_1 едет в бок стоящего ground_2 (ground_2 повёрнут на 90 градусов)
  rotate_near  - ground_1 вращается рядом (0.45 м) со стоящим ground_2
Печатает минимальное расстояние между центрами и конечное.
"""
import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

CMDS = {   # сценарий -> {робот: (v, w)}
    'head_static': {1: (0.5, 0.0)},
    'head_both': {1: (0.5, 0.0), 2: (0.5, 0.0)},
    'from_behind': {1: (0.5, 0.0)},
    'reverse_into': {1: (-0.3, 0.0)},
    'side': {1: (0.5, 0.0)},
    'rotate_near': {1: (0.0, 1.0)},
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('scenario', choices=sorted(CMDS))
    ap.add_argument('--time', type=float, default=12.0)
    a, _ = ap.parse_known_args()
    rclpy.init()
    n = Node('collision_test', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    pos = {}
    for i in (1, 2):
        n.create_subscription(Odometry, f'/ground_{i}/odom', lambda m, i=i: pos.__setitem__(
            i, (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
    pubs = {i: n.create_publisher(Twist, f'/ground_{i}/cmd_vel_nav', 10) for i in (1, 2)}
    while rclpy.ok() and len(pos) < 2:
        rclpy.spin_once(n, timeout_sec=0.1)
    t0 = n.get_clock().now().nanoseconds * 1e-9
    d0 = math.dist(pos[1], pos[2])
    best, end = d0, d0
    # ждём, пока монитор станет активным: первые секунды команды могут не пройти (это не важно для проверки)
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.05)
        t = n.get_clock().now().nanoseconds * 1e-9 - t0
        if t > a.time:
            break
        for i, (v, w) in CMDS[a.scenario].items():
            m = Twist()
            m.linear.x, m.angular.z = v, w
            pubs[i].publish(m)
        end = math.dist(pos[1], pos[2])
        best = min(best, end)
    for p in pubs.values():
        p.publish(Twist())
    print(f'{a.scenario}: начальное {d0:.2f} м, минимальное {best:.2f} м, конечное {end:.2f} м')
    sys.exit(0 if best >= 0.6 else 1)


if __name__ == '__main__':
    main()

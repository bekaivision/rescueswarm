#!/usr/bin/env python3
"""Только для разработки: публикует постоянную карту OccupancyGrid на /map.

  const_map.py [--unknown] [--size 40] [--res 0.05] [--frame world]
По умолчанию вся карта свободна (0); с --unknown — вся неизвестна (-1).
Карта центрирована в начале координат. QoS: transient_local, как у map_server.
"""
import argparse

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--unknown', action='store_true')
    ap.add_argument('--size', type=float, default=40.0)
    ap.add_argument('--res', type=float, default=0.05)
    ap.add_argument('--frame', default='world')
    ap.add_argument('--topic', default='/map')
    a, _ = ap.parse_known_args()
    rclpy.init()
    node = Node('const_map')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(OccupancyGrid, a.topic, qos)
    cells = int(round(a.size / a.res))
    msg = OccupancyGrid()
    msg.header.frame_id = a.frame
    msg.info.resolution = a.res
    msg.info.width = msg.info.height = cells
    msg.info.origin.position.x = msg.info.origin.position.y = -a.size / 2.0
    msg.info.origin.orientation.w = 1.0
    msg.data = [-1 if a.unknown else 0] * (cells * cells)
    pub.publish(msg)
    node.get_logger().info(f'карта {a.size} x {a.size} м, {"неизвестна" if a.unknown else "свободна"}')
    rclpy.spin(node)


if __name__ == '__main__':
    main()

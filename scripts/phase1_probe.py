#!/usr/bin/env python3
"""Помощник check_phase1.sh (запускается внутри контейнера).
  phase1_probe.py move OUTDIR   : разные cmd_vel для ground_1..3, odom до/после
  phase1_probe.py frames OUTDIR : по одному кадру RGB и тепловизора -> PNG
"""
import json
import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

CMDS = {'ground_1': (0.3, 0.0), 'ground_2': (0.0, 0.5), 'ground_3': (0.0, 0.0)}  # (v, w)
DUR = 6.0


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def spin_for(node, sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def move(out):
    node = Node('phase1_move')
    odom = {}
    for ns in CMDS:
        node.create_subscription(Odometry, f'/{ns}/odom',
                                 lambda m, ns=ns: odom.__setitem__(ns, m.pose.pose), 10)
    pubs = {ns: node.create_publisher(Twist, f'/{ns}/cmd_vel', 10) for ns in CMDS}
    spin_for(node, 2.0)
    before = {ns: odom[ns] for ns in CMDS}
    end = time.time() + DUR
    while time.time() < end:
        for ns, (v, w) in CMDS.items():
            t = Twist(); t.linear.x = v; t.angular.z = w
            pubs[ns].publish(t)
        spin_for(node, 0.1)
    for p in pubs.values():
        p.publish(Twist())
    spin_for(node, 1.5)
    res = {}
    for ns in CMDS:
        a, b = before[ns], odom[ns]
        dx, dy = b.position.x - a.position.x, b.position.y - a.position.y
        dyaw = math.degrees(yaw_of(b.orientation) - yaw_of(a.orientation))
        dyaw = (dyaw + 180) % 360 - 180
        res[ns] = {'cmd_v': CMDS[ns][0], 'cmd_w': CMDS[ns][1],
                   'dist_m': round(math.hypot(dx, dy), 3), 'dx': round(dx, 3), 'dy': round(dy, 3),
                   'dyaw_deg': round(dyaw, 1)}
    res['_expected'] = {'ground_1': f'{CMDS["ground_1"][0]*DUR:.1f} m вперёд, поворот ~0',
                        'ground_2': f'{math.degrees(CMDS["ground_2"][1]*DUR):.0f} град, смещение ~0',
                        'ground_3': 'стоит'}
    open(f'{out}/independent_motion.json', 'w').write(json.dumps(res, indent=2, ensure_ascii=False))
    print(json.dumps(res, indent=2, ensure_ascii=False))
    node.destroy_node()


def to_png(arr, path):
    import cv2
    cv2.imwrite(path, arr)


def frames(out):
    node = Node('phase1_frames')
    got = {}
    for name, topic in (('rgb', '/ground_1/rgb/image_raw'), ('thermal', '/ground_1/thermal/image_raw')):
        node.create_subscription(Image, topic, lambda m, n=name: got.setdefault(n, m),
                                 qos_profile_sensor_data)
    end = time.time() + 20
    while len(got) < 2 and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    for n, m in got.items():
        if m.encoding == 'rgb8':
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
            to_png(a[:, :, ::-1], f'{out}/ground_1_rgb.png')
            print('rgb', a.shape, 'mean', a.mean(), 'std', a.std(), 'min', a.min(), 'max', a.max())
        else:  # mono16, температурная шкала Gazebo
            a = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
            print('thermal raw', a.shape, m.encoding, 'min', a.min(), 'max', a.max(), 'mean', a.mean(),
                  'unique', len(np.unique(a)))
            lo, hi = float(a.min()), float(a.max())
            v = ((a - lo) / max(hi - lo, 1) * 255).astype(np.uint8)
            import cv2
            cv2.imwrite(f'{out}/ground_1_thermal_raw16.png', a)
            cv2.imwrite(f'{out}/ground_1_thermal.png', cv2.applyColorMap(v, cv2.COLORMAP_INFERNO))
    if len(got) < 2:
        print('FAIL: не получены кадры', set(got)); sys.exit(1)
    node.destroy_node()


if __name__ == '__main__':
    rclpy.init()
    {'move': move, 'frames': frames}[sys.argv[1]](sys.argv[2])
    rclpy.shutdown()

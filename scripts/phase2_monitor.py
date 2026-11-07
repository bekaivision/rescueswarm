#!/usr/bin/env python3
"""Фаза 2: монитор безопасности и «чистоты управления» во время прогона (внутри контейнера).

  phase2_monitor.py OUT_JSON WORLD_DIR [N=3]
Подписывается на /ground_i/odom (истинная поза), считает:
  - минимальное расстояние между центрами роботов;
  - число «касаний стен» (центр ближе 0.15 м к занятой клетке gt_map.pgm; считаются заходы);
  - путь каждого робота;
  - кто публикует /ground_i/cmd_vel и /ground_i/cmd_vel_nav (узлы, раз в 2 с, объединение за весь прогон):
    в cmd_vel должен писать только collision_monitor, в cmd_vel_nav - controller/behaviors/explorer.
Пишет OUT_JSON каждые 5 с (работает, пока его не убьют).
"""
import itertools
import json
import math
import sys

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.ndimage import distance_transform_edt

TOUCH_M = 0.15


class Monitor(Node):
    def __init__(self, out, world, n):
        super().__init__('phase2_monitor', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.out, self.n = out, n
        self.robots = [f'ground_{i}' for i in range(1, n + 1)]
        # карта препятствий и расстояние до ближайшей занятой клетки
        from PIL import Image
        m = yaml.safe_load(open(f'{world}/gt_map.yaml'))
        img = np.array(Image.open(f'{world}/gt_map.pgm'))
        occ = img < 128
        self.dist = distance_transform_edt(~occ) * float(m['resolution'])
        self.res = float(m['resolution'])
        self.ox, self.oy = float(m['origin'][0]), float(m['origin'][1])
        self.h = img.shape[0]
        self.pos = {}
        self.last = {}
        self.path = {r: 0.0 for r in self.robots}
        self.touch = {r: 0 for r in self.robots}
        self.in_touch = {r: False for r in self.robots}
        self.min_pair = {'dist': 1e9, 'pair': '', 'sim_t': 0.0}
        self.pubs = {r: set() for r in self.robots}
        self.pubs_nav = {r: set() for r in self.robots}
        # журнал поз (раз в 0.5 сим. с) для разбора сближений: <OUT_JSON без .json>_poses.csv
        self.poses_f = open(out.rsplit('.', 1)[0] + '_poses.csv', 'w')
        self.poses_f.write('t,robot,x,y,yaw\n')
        self.last_log = {r: -1e9 for r in self.robots}
        for r in self.robots:
            self.create_subscription(Odometry, f'/{r}/odom', lambda m, r=r: self._odom(r, m), qos_profile_sensor_data)
        self.create_timer(2.0, self._pubs)
        self.create_timer(5.0, self._write)

    def _wall_dist(self, x, y):
        col = int((x - self.ox) / self.res)
        row = self.h - 1 - int((y - self.oy) / self.res)
        if 0 <= row < self.h and 0 <= col < self.dist.shape[1]:
            return float(self.dist[row, col])
        return 1e9

    def _odom(self, r, m):
        x, y = m.pose.pose.position.x, m.pose.pose.position.y
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if r in self.last:
            self.path[r] += math.hypot(x - self.last[r][0], y - self.last[r][1])
        self.last[r] = (x, y)
        self.pos[r] = (x, y)
        if t - self.last_log[r] >= 0.5:
            self.last_log[r] = t
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.poses_f.write(f'{t:.1f},{r},{x:.2f},{y:.2f},{yaw:.2f}\n')
        touching = self._wall_dist(x, y) < TOUCH_M
        if touching and not self.in_touch[r]:
            self.touch[r] += 1
        self.in_touch[r] = touching
        for a, b in itertools.combinations(self.robots, 2):
            if a in self.pos and b in self.pos:
                d = math.hypot(self.pos[a][0] - self.pos[b][0], self.pos[a][1] - self.pos[b][1])
                if d < self.min_pair['dist']:
                    self.min_pair = {'dist': d, 'pair': f'{a}-{b}', 'sim_t': t}

    def _pubs(self):
        for r in self.robots:
            for i in self.get_publishers_info_by_topic(f'/{r}/cmd_vel'):
                ns = i.node_namespace.rstrip('/')
                self.pubs[r].add(f'{ns}/{i.node_name}')
            for i in self.get_publishers_info_by_topic(f'/{r}/cmd_vel_nav'):
                self.pubs_nav[r].add(f'{i.node_namespace.rstrip("/")}/{i.node_name}')

    def _write(self):
        d = {'min_robot_distance_m': round(self.min_pair['dist'], 3), 'min_pair': self.min_pair['pair'],
             'min_pair_sim_t': round(self.min_pair['sim_t'], 1),
             'path_m': {r: round(v, 1) for r, v in self.path.items()},
             'wall_touches': self.touch, 'cmd_vel_publishers': {r: sorted(v) for r, v in self.pubs.items()},
             'cmd_vel_nav_publishers': {r: sorted(v) for r, v in self.pubs_nav.items()}}
        json.dump(d, open(self.out, 'w'), indent=1, ensure_ascii=False)
        self.poses_f.flush()


def main():
    rclpy.init()
    node = Monitor(sys.argv[1], sys.argv[2], int(sys.argv[3]) if len(sys.argv) > 3 else 3)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    node._write()


if __name__ == '__main__':
    main()

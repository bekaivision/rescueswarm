#!/usr/bin/env python3
"""Запись сырья для видео Фазы 6 (внутри контейнера рядом с explore.launch в режиме comms:=true).

  phase6_rec.py OUT.npz [N=3]
Пишет по времени симуляции: карту базы /map (то, что видит спасатель) и собственную карту ground_1
/ground_1/map (то, что знает сам робот), оба раза раз в 1 с (int8, одна сетка); истинные позы
/ground_i/gt_odom (5 Гц); отметки людей /people/marks базы (раз в 1 с). Файл сохраняется при остановке
и каждые 30 с.
"""
import signal
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rs_interfaces.msg import PersonMarkArray


def yaw_of(q):
    return float(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


class Rec(Node):
    def __init__(self, out, n):
        super().__init__('phase6_rec', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.out = out
        self.robots = [f'ground_{i}' for i in range(1, n + 1)]
        self.maps, self.maps_own, self.map_t = [], [], []
        self.last_own = None
        self.pose = []                      # (t, idx, x, y, yaw)
        self.marks = []
        self.last = {}
        self.meta = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._map, qos)
        self.create_subscription(OccupancyGrid, '/ground_1/map', self._own, qos)
        for r in self.robots:
            self.create_subscription(Odometry, f'/{r}/gt_odom', lambda msg, r=r: self._odom(r, msg), 10)
        self.create_subscription(PersonMarkArray, '/people/marks', self._marks, 5)
        self.create_timer(30.0, self.save)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def due(self, key, dt):
        t = self.now()
        if t - self.last.get(key, -1e9) < dt:
            return None
        self.last[key] = t
        return t

    def _own(self, m):
        self.last_own = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)

    def _map(self, m):
        t = self.due('map', 0.95)
        if t is None:
            return
        i = m.info
        self.meta = np.array([i.resolution, i.origin.position.x, i.origin.position.y, i.width, i.height])
        d = np.array(m.data, dtype=np.int8).reshape(i.height, i.width)
        self.maps.append(d)
        g = self.last_own if self.last_own is not None and self.last_own.shape == d.shape \
            else np.full(d.shape, -1, np.int8)
        self.maps_own.append(g.copy())
        self.map_t.append(t)

    def _odom(self, r, m):
        t = self.due(('p', r), 0.19)
        if t is not None:
            p = m.pose.pose.position
            self.pose.append((t, self.robots.index(r), p.x, p.y, yaw_of(m.pose.pose.orientation)))

    def _marks(self, m):
        t = self.due('marks', 0.95)
        if t is None:
            return
        for pm in m.marks:
            st = pm.status_time.sec + pm.status_time.nanosec * 1e-9
            self.marks.append((t, int(pm.id.split('_')[1]), pm.status, pm.position.x, pm.position.y, st,
                               pm.confidence))

    def save(self):
        if not self.maps:
            return
        np.savez_compressed(self.out, maps=np.array(self.maps), maps_own=np.array(self.maps_own),
                            map_t=np.array(self.map_t), meta=self.meta, pose=np.array(self.pose),
                            marks=np.array(self.marks).reshape(-1, 7))
        self.get_logger().info(f'сохранено: {len(self.maps)} карт, {len(self.pose)} поз -> {self.out}')


def main():
    rclpy.init()
    a = sys.argv
    node = Rec(a[1], int(a[2]) if len(a) > 2 else 3)
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    node.save()


if __name__ == '__main__':
    main()

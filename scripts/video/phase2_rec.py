#!/usr/bin/env python3
"""Запись сырья для видео Фазы 2 (запускается внутри контейнера рядом с explore.launch).

  phase2_rec.py OUT.npz [N=3]
Пишет по времени симуляции: карту /map (раз в 1 с), позы роботов (истинная одометрия, 5 Гц),
текущие цели (из ставок /exploration/bids), клетки-фронтиры (маркер ground_1, раз в 1 с).
Файл сохраняется при остановке (SIGINT/SIGTERM) и каждые 30 с.
"""
import signal
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rs_interfaces.msg import ExplorationBid
from visualization_msgs.msg import MarkerArray


class Rec(Node):
    def __init__(self, out, n):
        super().__init__('phase2_rec', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.out = out
        self.robots = [f'ground_{i}' for i in range(1, n + 1)]
        self.maps, self.map_t = [], []
        self.pose = []                 # (t, robot_idx, x, y, yaw)
        self.goal = []                 # (t, robot_idx, has_goal, x, y)
        self.front = []                # (t, массив N x 2)
        self.last_pose = {r: -1e9 for r in self.robots}
        self.last_map = -1e9
        self.last_front = -1e9
        self.meta = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._map, qos)
        for r in self.robots:
            self.create_subscription(Odometry, f'/{r}/odom', lambda m, r=r: self._odom(r, m), 10)
        self.create_subscription(ExplorationBid, '/exploration/bids', self._bid, 20)
        self.create_subscription(MarkerArray, '/ground_1/explorer/frontiers', self._front, 1)
        self.create_timer(30.0, self.save)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _map(self, m):
        t = self.now()
        if t - self.last_map < 0.95:
            return
        self.last_map = t
        i = m.info
        self.meta = np.array([i.resolution, i.origin.position.x, i.origin.position.y, i.width, i.height])
        self.maps.append(np.array(m.data, dtype=np.int8).reshape(i.height, i.width))
        self.map_t.append(t)

    def _odom(self, r, m):
        t = self.now()
        if t - self.last_pose[r] < 0.19:
            return
        self.last_pose[r] = t
        q = m.pose.pose.orientation
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose.append((t, self.robots.index(r), m.pose.pose.position.x, m.pose.pose.position.y, yaw))

    def _bid(self, m):
        if m.robot_id in self.robots:
            g = m.current_goal
            self.goal.append((self.now(), self.robots.index(m.robot_id), float(m.has_goal), g.x, g.y))

    def _front(self, m):
        t = self.now()
        if t - self.last_front < 0.95:
            return
        for mk in m.markers:
            if mk.ns == 'frontier_cells':
                self.last_front = t
                self.front.append((t, np.array([[p.x, p.y] for p in mk.points], dtype=np.float32).reshape(-1, 2)))

    def save(self):
        if not self.maps:
            return
        ft = np.array([t for t, _ in self.front])
        fc = np.array([len(a) for _, a in self.front])
        fp = np.concatenate([a for _, a in self.front]) if self.front else np.zeros((0, 2), np.float32)
        np.savez_compressed(self.out, maps=np.array(self.maps), map_t=np.array(self.map_t), meta=self.meta,
                            pose=np.array(self.pose), goal=np.array(self.goal),
                            front_t=ft, front_cnt=fc, front_pts=fp)
        self.get_logger().info(f'сохранено: {len(self.maps)} карт, {len(self.pose)} поз -> {self.out}')


def main():
    rclpy.init()
    node = Rec(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 3)
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

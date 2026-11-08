#!/usr/bin/env python3
"""Запись сырья для видео Фазы 3 (запускается внутри контейнера рядом с explore.launch).

  phase3_rec.py OUT.npz [N=3]
Пишет по времени симуляции: карту /map (раз в 1 с), позы роботов (истинная одометрия, 5 Гц), текущие цели
(из ставок /exploration/bids), отметки людей /people/marks (раз в 1 с), тепловые кадры каждого робота
(mono16, 0.01 K на отсчёт, не чаще раза в 0.45 с). Файл сохраняется при остановке и каждые 30 с.
"""
import signal
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rs_interfaces.msg import ExplorationBid, PersonMarkArray
from sensor_msgs.msg import Image


class Rec(Node):
    def __init__(self, out, n):
        super().__init__('phase3_rec', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.out = out
        self.robots = [f'ground_{i}' for i in range(1, n + 1)]
        self.maps, self.map_t = [], []
        self.pose, self.goal = [], []      # (t, robot_idx, x, y, yaw) / (t, robot_idx, has_goal, x, y)
        self.marks = []                    # (t, id_num, status, x, y, status_time, confidence)
        self.th, self.th_t, self.th_r = [], [], []
        self.last_pose = {r: -1e9 for r in self.robots}
        self.last_th = {r: -1e9 for r in self.robots}
        self.last_map = self.last_marks = -1e9
        self.meta = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._map, qos)
        for r in self.robots:
            self.create_subscription(Odometry, f'/{r}/odom', lambda m, r=r: self._odom(r, m), 10)
            self.create_subscription(Image, f'/{r}/thermal/image_raw', lambda m, r=r: self._th(r, m),
                                     qos_profile_sensor_data)
        self.create_subscription(ExplorationBid, '/exploration/bids', self._bid, 20)
        self.create_subscription(PersonMarkArray, '/people/marks', self._marks, 5)
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

    def _marks(self, m):
        t = self.now()
        if t - self.last_marks < 0.95:
            return
        self.last_marks = t
        for pm in m.marks:
            st = pm.status_time.sec + pm.status_time.nanosec * 1e-9
            self.marks.append((t, int(pm.id.split('_')[1]), pm.status, pm.position.x, pm.position.y, st,
                               pm.confidence))

    def _th(self, r, m):
        t = self.now()
        if t - self.last_th[r] < 0.45:
            return
        self.last_th[r] = t
        dt = '>u2' if m.is_bigendian else '<u2'
        raw = np.frombuffer(m.data, dtype=dt).reshape(m.height, m.step // 2)[:, :m.width]
        self.th.append(raw.astype(np.uint16))
        self.th_t.append(t)
        self.th_r.append(self.robots.index(r))

    def save(self):
        if not self.maps:
            return
        np.savez_compressed(self.out, maps=np.array(self.maps), map_t=np.array(self.map_t), meta=self.meta,
                            pose=np.array(self.pose), goal=np.array(self.goal),
                            marks=np.array(self.marks).reshape(-1, 7),
                            th=np.array(self.th), th_t=np.array(self.th_t), th_r=np.array(self.th_r))
        self.get_logger().info(f'сохранено: {len(self.maps)} карт, {len(self.pose)} поз, {len(self.th)} тепловых кадров -> {self.out}')


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

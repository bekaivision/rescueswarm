#!/usr/bin/env python3
"""Запись сырья для видео Фазы 5 (внутри контейнера рядом с explore.launch, дроны включены).

  phase5_rec.py OUT.npz [N=3] [M=2]
Пишет по времени симуляции: общую карту /map и наземный слой /map_ground (раз в 1 с; клетки, известные
только сверху = известные в /map и неизвестные в /map_ground), позы наземных роботов /ground_i/pose (5 Гц),
позы дронов /drone_i/pose (x, y, z, курс; 5 Гц), цели наземных роботов (/exploration/bids) и дронов
(/exploration/air_bids), отметки людей /people/marks (раз в 1 с), тепловые кадры дронов (вид сверху,
mono16, 0.01 K на отсчёт, не чаще раза в 0.45 с). Файл сохраняется при остановке и каждые 30 с.
Формат совместим с phase3_render.Data (maps, pose, goal, marks, th*), дроны в отдельных массивах.
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


def yaw_of(q):
    return float(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


class Rec(Node):
    def __init__(self, out, n, m):
        super().__init__('phase5_rec', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.out = out
        self.robots = [f'ground_{i}' for i in range(1, n + 1)]
        self.drones = [f'drone_{i}' for i in range(1, m + 1)]
        self.maps, self.map_t, self.maps_g = [], [], []
        self.last_ground = None
        self.pose, self.goal = [], []       # (t, idx, x, y, yaw) / (t, idx, has_goal, x, y)
        self.dpose, self.dgoal = [], []     # (t, idx, x, y, z, yaw) / (t, idx, has_goal, x, y)
        self.marks = []
        self.th, self.th_t, self.th_r = [], [], []
        self.last = {}
        self.meta = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._map, qos)
        self.create_subscription(OccupancyGrid, '/map_ground', self._map_g, qos)
        for r in self.robots:
            self.create_subscription(Odometry, f'/{r}/pose', lambda msg, r=r: self._odom(r, msg), 10)
        for r in self.drones:
            self.create_subscription(Odometry, f'/{r}/pose', lambda msg, r=r: self._dodom(r, msg), 10)
            self.create_subscription(Image, f'/{r}/thermal/image_raw', lambda msg, r=r: self._th(r, msg),
                                     qos_profile_sensor_data)
        self.create_subscription(ExplorationBid, '/exploration/bids', self._bid, 20)
        self.create_subscription(ExplorationBid, '/exploration/air_bids', self._dbid, 20)
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

    def _map_g(self, m):
        self.last_ground = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)

    def _map(self, m):
        t = self.due('map', 0.95)
        if t is None:
            return
        i = m.info
        self.meta = np.array([i.resolution, i.origin.position.x, i.origin.position.y, i.width, i.height])
        d = np.array(m.data, dtype=np.int8).reshape(i.height, i.width)
        self.maps.append(d)
        g = self.last_ground if self.last_ground is not None and self.last_ground.shape == d.shape \
            else np.full(d.shape, -1, np.int8)
        self.maps_g.append(g.copy())
        self.map_t.append(t)

    def _odom(self, r, m):
        t = self.due(('p', r), 0.19)
        if t is not None:
            p = m.pose.pose.position
            self.pose.append((t, self.robots.index(r), p.x, p.y, yaw_of(m.pose.pose.orientation)))

    def _dodom(self, r, m):
        t = self.due(('d', r), 0.19)
        if t is not None:
            p = m.pose.pose.position
            self.dpose.append((t, self.drones.index(r), p.x, p.y, p.z, yaw_of(m.pose.pose.orientation)))

    def _bid(self, m):
        if m.robot_id in self.robots:
            g = m.current_goal
            self.goal.append((self.now(), self.robots.index(m.robot_id), float(m.has_goal), g.x, g.y))

    def _dbid(self, m):
        if m.robot_id in self.drones:
            g = m.current_goal
            self.dgoal.append((self.now(), self.drones.index(m.robot_id), float(m.has_goal), g.x, g.y))

    def _marks(self, m):
        t = self.due('marks', 0.95)
        if t is None:
            return
        for pm in m.marks:
            st = pm.status_time.sec + pm.status_time.nanosec * 1e-9
            self.marks.append((t, int(pm.id.split('_')[1]), pm.status, pm.position.x, pm.position.y, st,
                               pm.confidence))

    def _th(self, r, m):
        t = self.due(('th', r), 0.45)
        if t is None:
            return
        dt = '>u2' if m.is_bigendian else '<u2'
        raw = np.frombuffer(m.data, dtype=dt).reshape(m.height, m.step // 2)[:, :m.width]
        self.th.append(raw.astype(np.uint16))
        self.th_t.append(t)
        self.th_r.append(self.drones.index(r))

    def save(self):
        if not self.maps:
            return
        np.savez_compressed(self.out, maps=np.array(self.maps), maps_g=np.array(self.maps_g),
                            map_t=np.array(self.map_t), meta=self.meta,
                            pose=np.array(self.pose), goal=np.array(self.goal).reshape(-1, 5),
                            dpose=np.array(self.dpose).reshape(-1, 6), dgoal=np.array(self.dgoal).reshape(-1, 5),
                            marks=np.array(self.marks).reshape(-1, 7),
                            th=np.array(self.th), th_t=np.array(self.th_t), th_r=np.array(self.th_r))
        self.get_logger().info(f'сохранено: {len(self.maps)} карт, {len(self.pose)} поз, {len(self.dpose)} поз дронов, '
                               f'{len(self.th)} тепловых кадров -> {self.out}')


def main():
    rclpy.init()
    a = sys.argv
    node = Rec(a[1], int(a[2]) if len(a) > 2 else 3, int(a[3]) if len(a) > 3 else 2)
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

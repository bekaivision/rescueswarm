"""Узел shared_map: одна общая карта занятости /map для всех роботов (кадр world).

Карта строится по лидарам /<robot>/lidar/points и TF на момент снимка (истинные позы,
ошибки SLAM нет). Занятость по log-odds, свободное пространство лучами от лидара.
Параметры: robots (список), gt_file (ground_truth.yaml: границы карты) или bounds
[xmin, ymin, xmax, ymax], resolution, z_min, z_max, scan_rate, map_rate.
"""
import time

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from rs_mapping.grid_update import GridUpdater, cloud_to_array


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class SharedMap(Node):
    def __init__(self):
        super().__init__('shared_map')
        self.declare_parameter('robots', ['ground_1', 'ground_2', 'ground_3'])
        self.declare_parameter('gt_file', '')
        self.declare_parameter('bounds', [-13.0, -11.0, 13.0, 11.0])
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('global_frame', 'world')
        self.declare_parameter('z_min', 0.05)
        self.declare_parameter('z_max', 1.5)
        self.declare_parameter('scan_rate', 3.0)   # Гц на робота
        self.declare_parameter('map_rate', 1.0)    # Гц публикации /map
        self.declare_parameter('robot_clear', 0.4)  # радиус отбрасывания точек вокруг роботов, м

        self.robots = list(self.get_parameter('robots').value)
        res = float(self.get_parameter('resolution').value)
        gt = self.get_parameter('gt_file').value
        if gt:
            with open(gt) as f:
                b = yaml.safe_load(f)['bounds']
            bounds = [b['xmin'], b['ymin'], b['xmax'], b['ymax']]
        else:
            bounds = list(self.get_parameter('bounds').value)
        self.frame = self.get_parameter('global_frame').value
        w = int(round((bounds[2] - bounds[0]) / res))
        h = int(round((bounds[3] - bounds[1]) / res))
        self.up = GridUpdater(
            w, h, res, bounds[0], bounds[1],
            z_min=float(self.get_parameter('z_min').value),
            z_max=float(self.get_parameter('z_max').value),
            robot_clear=float(self.get_parameter('robot_clear').value))
        self.get_logger().info(f'карта {w}x{h} клеток, {res} м, origin=({bounds[0]}, {bounds[1]}), роботы: {self.robots}')

        self.tf_buffer = Buffer(cache_time=Duration(seconds=20))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.pending = {r: None for r in self.robots}
        self.last_proc = {r: -1.0 for r in self.robots}
        self.poses = {}
        self.offsets = None
        sensor_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        for r in self.robots:
            self.create_subscription(PointCloud2, f'/{r}/lidar/points',
                                     lambda m, r=r: self._on_cloud(r, m), sensor_qos)
            self.create_subscription(Odometry, f'/{r}/odom',
                                     lambda m, r=r: self._on_odom(r, m), 10)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.create_timer(1.0 / float(self.get_parameter('scan_rate').value) / len(self.robots),
                          self._process)
        self.create_timer(1.0 / float(self.get_parameter('map_rate').value), self._publish)
        self.n_scans = 0
        self.t_cpu = 0.0
        self.t_data = 0.0
        self.t_pub = 0.0
        self.create_timer(30.0, self._stats)
        # сразу публикуем пустую карту: без неё Nav2 не активируется
        self._publish()

    def _on_odom(self, r, msg):
        p = msg.pose.pose.position
        self.poses[r] = (p.x, p.y)

    def _on_cloud(self, r, msg):
        self.pending[r] = msg

    def _process(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        min_dt = 1.0 / float(self.get_parameter('scan_rate').value)
        for r in self.robots:
            msg = self.pending[r]
            if msg is None or now - self.last_proc[r] < min_dt:
                continue
            stamp = Time.from_msg(msg.header.stamp)
            try:
                tf = self.tf_buffer.lookup_transform(self.frame, msg.header.frame_id, stamp)
            except TransformException:
                # TF ещё не дошёл: ждём; слишком старые снимки выбрасываем
                if now - stamp.nanoseconds * 1e-9 > 1.5:
                    self.pending[r] = None
                continue
            self.pending[r] = None
            self.last_proc[r] = now
            t0 = time.process_time()
            if self.offsets is None:
                self.offsets = [next(f.offset for f in msg.fields if f.name == n) for n in ('x', 'y', 'z')]
            pts = cloud_to_array(msg.data, msg.width, msg.height, msg.point_step, self.offsets)
            T = np.eye(4)
            T[:3, :3] = quat_to_matrix(tf.transform.rotation)
            tr = tf.transform.translation
            T[:3, 3] = (tr.x, tr.y, tr.z)
            self.up.update(pts, T, [v for k, v in self.poses.items() if k != r], self.poses.get(r))
            self.n_scans += 1
            self.t_cpu += time.process_time() - t0

    def _publish(self):
        m = OccupancyGrid()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        m.info.resolution = self.up.res
        m.info.width = self.up.w
        m.info.height = self.up.h
        m.info.origin.position.x = self.up.ox
        m.info.origin.position.y = self.up.oy
        m.info.origin.orientation.w = 1.0
        t0 = time.process_time()
        m.data = self.up.to_grid_data().tolist()
        self.t_data += time.process_time() - t0
        t0 = time.process_time()
        self.pub.publish(m)
        self.t_pub += time.process_time() - t0

    def _stats(self):
        n = max(self.n_scans, 1)
        self.get_logger().info(f'сканов {self.n_scans}, {1000 * self.t_cpu / n:.1f} мс CPU на скан, '
                               f'известно {100.0 * self.up.seen.mean():.1f} % карты; CPU всего: сканы {self.t_cpu:.1f} с, tolist {self.t_data:.1f} с, publish {self.t_pub:.1f} с')


def main():
    rclpy.init()
    node = SharedMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

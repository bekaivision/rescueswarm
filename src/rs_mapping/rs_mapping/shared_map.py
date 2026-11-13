"""Узел shared_map: одна общая карта занятости /map для всех роботов (кадр world).

Карта строится по лидарам /<robot>/lidar/points и TF на момент снимка (истинные позы,
ошибки SLAM нет). Занятость по log-odds, свободное пространство лучами от лидара.
Параметры: robots (список), gt_file (ground_truth.yaml: границы карты) или bounds
[xmin, ymin, xmax, ymax], resolution, z_min, z_max, scan_rate, map_rate.
Дроны (Фаза 5): air_robots (список), air_scan_rate, air_obst_h, air_free_h, air_max_h,
free_z_max. При непустом air_robots кроме /map публикуются /map_ground и /map_air.
Режим слоёв (Фаза 6, layers=true): у каждого робота свой слой, который строят только его датчики;
вместо /map, /map_ground, /map_air публикуются /<робот>/map_layer (MapLayer, int8, см. rs_mapping.layers)
и /map_collective (слияние всех слоёв, только для оценки).
"""
import array
import time

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rs_interfaces.msg import MapLayer
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from rs_mapping import layers as L
from rs_mapping.air_update import AirLayer, fuse
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
        self.declare_parameter('robots', [], ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('gt_file', '')
        self.declare_parameter('bounds', [-13.0, -11.0, 13.0, 11.0])
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('global_frame', 'world')
        self.declare_parameter('z_min', 0.05)
        self.declare_parameter('z_max', 1.5)
        self.declare_parameter('scan_rate', 3.0)   # Гц на робота
        self.declare_parameter('map_rate', 1.0)    # Гц публикации /map
        self.declare_parameter('robot_clear', 0.4)  # радиус отбрасывания точек вокруг роботов, м
        # dynamic_typing: пустой список из launch не имеет типа, а типизированный параметр
        # без значения бросает ParameterUninitializedException
        self.declare_parameter('air_robots', [], ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('air_scan_rate', 2.0)  # Гц на дрона
        self.declare_parameter('air_obst_h', 0.15)    # h >= этого: занято, м над полом
        self.declare_parameter('air_free_h', 0.08)    # h <= этого: свободно
        self.declare_parameter('air_max_h', 3.0)      # точки выше отбрасываем
        self.declare_parameter('air_max_off_nadir_deg', 35.0)  # конус от вертикали; 0 = весь кадр
        self.declare_parameter('air_floor_plane', True)  # пол в кадре - плоскость (поправка наклона дрона)
        self.declare_parameter('free_z_max', 2.5)     # выше этого лидар не режет свободное, м
        self.declare_parameter('layers', False)       # Фаза 6: слой на каждого робота вместо общей карты

        self.robots = list(self.get_parameter('robots').value or [])
        if not self.robots:
            raise RuntimeError('параметр robots пуст: список роботов задаёт launch (explore.launch.py / slam.launch.py), запасного списка нет')
        self.air_robots = list(self.get_parameter('air_robots').value or [])
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
            robot_clear=float(self.get_parameter('robot_clear').value),
            free_z_max=float(self.get_parameter('free_z_max').value))
        self.layers = bool(self.get_parameter('layers').value)

        def make_air():
            return AirLayer(
                w, h, res, bounds[0], bounds[1],
                obst_h=float(self.get_parameter('air_obst_h').value),
                free_h=float(self.get_parameter('air_free_h').value),
                max_h=float(self.get_parameter('air_max_h').value),
                max_off_nadir_deg=float(self.get_parameter('air_max_off_nadir_deg').value),
                floor_plane=bool(self.get_parameter('air_floor_plane').value))
        self.air = None
        if self.air_robots and not self.layers:
            self.air = make_air()
        # режим слоёв: свой GridUpdater на каждого наземного и свой AirLayer на каждого дрона
        self.g_layers = {}
        self.a_layers = {}
        self.versions = {}
        if self.layers:
            for r in self.robots:
                self.g_layers[r] = GridUpdater(
                    w, h, res, bounds[0], bounds[1],
                    z_min=float(self.get_parameter('z_min').value),
                    z_max=float(self.get_parameter('z_max').value),
                    robot_clear=float(self.get_parameter('robot_clear').value),
                    free_z_max=float(self.get_parameter('free_z_max').value))
            for r in self.air_robots:
                self.a_layers[r] = make_air()
            self.versions = {r: 0 for r in self.robots + self.air_robots}
        self.get_logger().info(f'карта {w}x{h} клеток, {res} м, origin=({bounds[0]}, {bounds[1]}), роботы: {self.robots}')

        self.tf_buffer = Buffer(cache_time=Duration(seconds=20))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.pending = {r: None for r in self.robots}
        self.last_proc = {r: -1.0 for r in self.robots}
        self.poses = {}
        self.drone_poses = {}
        self.air_pending = {r: None for r in self.air_robots}
        self.air_last = {r: -1.0 for r in self.air_robots}
        self.offsets = None
        sensor_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        for r in self.robots:
            self.create_subscription(PointCloud2, f'/{r}/lidar/points',
                                     lambda m, r=r: self._on_cloud(r, m), sensor_qos)
            self.create_subscription(Odometry, f'/{r}/pose',
                                     lambda m, r=r: self._on_odom(r, m), 10)
        # облако глубины дрона ~300 КБ: с BEST_EFFORT под нагрузкой теряются фрагменты DDS
        # (доходило 1-3 Гц из 5), поэтому RELIABLE
        air_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        for r in self.air_robots:
            self.create_subscription(PointCloud2, f'/{r}/depth/points',
                                     lambda m, r=r: self._on_air_cloud(r, m), air_qos)
            self.create_subscription(Odometry, f'/{r}/pose',
                                     lambda m, r=r: self._on_drone_odom(r, m), 10)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        if self.layers:
            layer_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE,
                                   history=HistoryPolicy.KEEP_LAST)
            self.pub_layer = {r: self.create_publisher(MapLayer, f'/{r}/map_layer', layer_qos)
                              for r in self.robots + self.air_robots}
            self.pub_coll = self.create_publisher(OccupancyGrid, '/map_collective', map_qos)
        else:
            self.pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.create_timer(1.0 / float(self.get_parameter('scan_rate').value) / len(self.robots),
                          self._process)
        if self.air_robots:
            if not self.layers:
                self.pub_ground = self.create_publisher(OccupancyGrid, '/map_ground', map_qos)
                self.pub_air = self.create_publisher(OccupancyGrid, '/map_air', map_qos)
            self.create_timer(1.0 / float(self.get_parameter('air_scan_rate').value)
                              / len(self.air_robots), self._process_air)
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

    def _on_drone_odom(self, r, msg):
        p = msg.pose.pose.position
        self.drone_poses[r] = (p.x, p.y, p.z)

    def _on_air_cloud(self, r, msg):
        self.air_pending[r] = msg

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
            up = self.g_layers[r] if self.layers else self.up
            up.update(pts, T, [v for k, v in self.poses.items() if k != r], self.poses.get(r),
                           list(self.drone_poses.values()))
            self.n_scans += 1
            self.t_cpu += time.process_time() - t0

    def _process_air(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        min_dt = 1.0 / float(self.get_parameter('air_scan_rate').value)
        for r in self.air_robots:
            msg = self.air_pending[r]
            if msg is None or now - self.air_last[r] < min_dt:
                continue
            stamp = Time.from_msg(msg.header.stamp)
            try:
                tf = self.tf_buffer.lookup_transform(self.frame, msg.header.frame_id, stamp)
            except TransformException:
                if now - stamp.nanoseconds * 1e-9 > 1.5:
                    self.air_pending[r] = None
                continue
            self.air_pending[r] = None
            self.air_last[r] = now
            t0 = time.process_time()
            offs = [next(f.offset for f in msg.fields if f.name == n) for n in ('x', 'y', 'z')]
            pts = cloud_to_array(msg.data, msg.width, msg.height, msg.point_step, offs)
            R = quat_to_matrix(tf.transform.rotation)
            tr = tf.transform.translation
            flat = pts.reshape(-1, 3).astype(np.float64)
            world = flat @ R.T + np.array([tr.x, tr.y, tr.z])
            air = self.a_layers[r] if self.layers else self.air
            air.update(world, list(self.poses.values()),
                            [(v[0], v[1]) for k, v in self.drone_poses.items() if k != r],
                            sensor_xyz=(tr.x, tr.y, tr.z))
            self.n_scans += 1
            self.t_cpu += time.process_time() - t0

    def _grid_msg(self, data):
        m = OccupancyGrid()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        m.info.resolution = self.up.res
        m.info.width = self.up.w
        m.info.height = self.up.h
        m.info.origin.position.x = self.up.ox
        m.info.origin.position.y = self.up.oy
        m.info.origin.orientation.w = 1.0
        m.data = array.array('b', data.astype(np.int8).tobytes()) if self.layers else data.tolist()
        return m

    def _publish_layers(self):
        """Режим слоёв: /<робот>/map_layer каждому и /map_collective (слияние всех слоёв, для оценки)."""
        now = self.get_clock().now().to_msg()
        qg, qa = [], []
        for kind, items, store in ((MapLayer.KIND_GROUND, self.g_layers, qg),
                                   (MapLayer.KIND_AIR, self.a_layers, qa)):
            for r, up in items.items():
                q = L.quantize(up.logodds, up.seen)
                store.append(q)
                self.versions[r] += 1
                m = MapLayer()
                m.header.stamp = now
                m.header.frame_id = self.frame
                m.source = r
                m.kind = kind
                m.version = self.versions[r]
                m.base_version = 0
                m.width, m.height = self.up.w, self.up.h
                m.resolution = float(self.up.res)
                m.origin_x, m.origin_y = float(self.up.ox), float(self.up.oy)
                m.encoding = MapLayer.ENC_RAW
                m.data = array.array('B', q.tobytes())
                self.pub_layer[r].publish(m)
        coll = L.fuse_grids(L.merge_grid(qg), L.merge_grid(qa))
        self.pub_coll.publish(self._grid_msg(coll))

    def _publish(self):
        if self.layers:
            self._publish_layers()
            return
        if self.air is not None:
            self.pub.publish(self._grid_msg(fuse(self.up, self.air)))
            self.pub_ground.publish(self._grid_msg(self.up.to_grid_data()))
            self.pub_air.publish(self._grid_msg(self.air.to_grid_data()))
            return
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

    def _known(self):
        """Доля известных клеток (в режиме слоёв: клетка известна, если её видел любой слой)."""
        if not self.layers:
            return self.up.seen.mean()
        seen = np.zeros_like(self.up.seen)
        for up in list(self.g_layers.values()) + list(self.a_layers.values()):
            seen |= up.seen
        return seen.mean()

    def _stats(self):
        n = max(self.n_scans, 1)
        self.get_logger().info(f'сканов {self.n_scans}, {1000 * self.t_cpu / n:.1f} мс CPU на скан, '
                               f'известно {100.0 * self._known():.1f} % карты; CPU всего: сканы {self.t_cpu:.1f} с, tolist {self.t_data:.1f} с, publish {self.t_pub:.1f} с')


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

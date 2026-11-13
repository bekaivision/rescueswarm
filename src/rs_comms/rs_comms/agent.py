"""Агент связи одного узла сети (Фаза 6): узел comms_agent.

Узел сети: наземный робот, дрон или база. Агент обменивается с соседями через comms_sim только
пакетами CommsPacket: маяки (что у кого есть), дельты слоёв карты, журнал наблюдений, ставки
аукциона, позы. Данные чужих источников ретранслирует (store and forward). Собирает локальную
карту узла и публикует её роботу.
"""
import array
import math
import os
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry, OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from rs_interfaces.msg import (CommsBeacon, CommsPacket, ExplorationBid, MapLayer, ObsBatch,
                               RgbDetection, ThermalDetection)

from rs_comms import map_sync as ms
from rs_comms.obs_log import ObsLog

RELIABLE_200 = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


class Agent(Node):
    def __init__(self):
        super().__init__('comms_agent')
        d = self.declare_parameter
        d('me', 'ground_1')
        d('kind', 'ground')                 # ground | air | base
        d('robots', [], ParameterDescriptor(dynamic_typing=True))            # обязателен, задаёт launch
        d('air_robots', [], ParameterDescriptor(dynamic_typing=True))
        d('gt_file', '')
        d('bounds', [-13.0, -11.0, 13.0, 11.0])   # запасной вариант, если gt_file пуст
        d('resolution', 0.05)
        d('log_dir', '')
        d('fusion', False)                  # читает onboard.py
        d('beacon_period', 1.0)
        d('map_period', 1.0)
        d('map_resend_s', 5.0)
        d('obs_period', 1.0)
        d('obs_resend_s', 4.0)
        d('max_backlog_s', 1.0)     # очередь данных радио дольше этого: синхронизацию не шлём
        d('obs_burst', 20)
        d('pose_period', 0.5)
        d('fleet_rate', 5.0)
        d('pose_ttl', 3.0)
        d('neighbor_ttl', 3.5)
        d('view_rate', 5.0)
        d('publish_period', 1.0)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.me = g('me')
        self.kind = g('kind')
        self.robots = list(g('robots') or [])
        if not self.robots:
            raise RuntimeError('параметр robots пуст: список роботов задаёт launch (explore.launch.py / slam.launch.py), запасного списка нет')
        self.air_robots = list(g('air_robots') or [])
        self.beacon_period = float(g('beacon_period'))
        self.map_resend_s = float(g('map_resend_s'))
        self.obs_resend_s = float(g('obs_resend_s'))
        self.obs_burst = int(g('obs_burst'))
        self.pose_ttl = float(g('pose_ttl'))
        self.neighbor_ttl = float(g('neighbor_ttl'))
        self.view_dt = 1.0 / max(float(g('view_rate')), 1e-3)
        self.max_backlog_s = float(g('max_backlog_s'))
        me = self.me
        is_ground = self.kind == 'ground'
        is_air = self.kind == 'air'
        is_base = self.kind == 'base'

        # геометрия карты как у shared_map
        res = float(g('resolution'))
        gt = g('gt_file')
        if gt:
            with open(gt) as f:
                b = yaml.safe_load(f)['bounds']
            bounds = [b['xmin'], b['ymin'], b['xmax'], b['ymax']]
            try:
                with open(gt) as f:
                    res = float(yaml.safe_load(f)['map']['resolution'])
            except (KeyError, TypeError):
                pass
        else:
            bounds = list(g('bounds'))
        self.res = res
        self.w = int(round((bounds[2] - bounds[0]) / res))
        self.h = int(round((bounds[3] - bounds[1]) / res))
        self.ox, self.oy = float(bounds[0]), float(bounds[1])
        self.ncell = self.w * self.h

        # радио
        self.tx = self.create_publisher(CommsPacket, '/comms/tx', RELIABLE_200)
        self.pub_have = self.create_publisher(CommsBeacon, f'/{me}/comms/have', 5)
        self.create_subscription(CommsPacket, f'/{me}/comms/rx', self._on_rx, RELIABLE_200)
        self.tx_seq = 0
        self.bytes_tx = {}
        self.neigh = {}         # имя -> время последнего пакета
        self.peers = {}         # имя -> dict(t, map, obs, nb) по последнему маяку
        # очередь своего радио (от comms_sim, как от сетевого интерфейса) + своя оценка между сообщениями
        self.backlog_s = 0.0
        self.bw = 0.0
        self.create_subscription(Float32MultiArray, f'/{me}/comms/status', self._on_status, 5)

        # карта
        self.layers = {}
        self.sent_map = {}      # источник -> (version, base, t)
        self._first_pub = True
        self.pub_map = {}
        if is_ground:
            self.pub_map['map'] = self.create_publisher(OccupancyGrid, f'/{me}/map', MAP_QOS)
        elif is_air:
            self.pub_map['air'] = self.create_publisher(OccupancyGrid, f'/{me}/map_air', MAP_QOS)
        else:
            self.pub_map['map'] = self.create_publisher(OccupancyGrid, '/map', MAP_QOS)
            self.pub_map['ground'] = self.create_publisher(OccupancyGrid, '/map_ground', MAP_QOS)
            self.pub_map['air'] = self.create_publisher(OccupancyGrid, '/map_air', MAP_QOS)
        if not is_base:
            self.create_subscription(MapLayer, f'/{me}/map_layer', self._on_own_layer, 5)

        # наблюдения
        self.log = ObsLog()
        self.obs_seq = 0
        self.obs_sent_t = {}    # (источник, seq) -> время последней отправки
        self.pend_det = []
        self.pend_rgb = []      # (RgbDetection, x, y, yaw)
        self.pend_view = []     # (t, x, y, yaw)
        self.last_view_t = -1e9
        self.pub_obs = self.create_publisher(ObsBatch, f'/{me}/comms/obs', RELIABLE_200)
        if not is_base:
            self.create_subscription(ThermalDetection, f'/{me}/perception/detections', self._on_det, 50)
        if is_ground:
            self.create_subscription(RgbDetection, '/perception/rgb_detections', self._on_rgb, 50)

        # позы
        self.pose = None        # (x, y, yaw) своя
        self.pose_msg = None
        self.peer_pose = {}     # имя -> (x, y, yaw, t получения)
        if is_ground:
            self.create_subscription(Odometry, f'/{me}/pose', self._on_pose, 10)
            self.pub_fleet = self.create_publisher(PoseArray, f'/{me}/fleet/poses', 5)

        # ставки
        self.bid_ch = self.bid_topic = None
        if is_ground:
            self.bid_ch, self.bid_topic = 'bid', f'/{me}/exploration/bids'
        elif is_air:
            self.bid_ch, self.bid_topic = 'air_bid', f'/{me}/exploration/air_bids'
        if self.bid_ch:
            self.pub_bid = self.create_publisher(ExplorationBid, self.bid_topic, 50)
            self.create_subscription(ExplorationBid, self.bid_topic, self._on_bid, 50)

        # файл журнала
        self.csv = None
        ld = g('log_dir')
        if ld:
            os.makedirs(os.path.join(ld, 'comms'), exist_ok=True)
            self.csv = open(os.path.join(ld, 'comms', f'agent_{me}.csv'), 'w')
            self.csv.write('t,source,map_have,obs_have,n_neighbors\n')

        self.create_timer(self.beacon_period, self._send_beacon)
        self.create_timer(float(g('map_period')), self._map_tick)
        self.create_timer(float(g('obs_period')), self._obs_tick)
        self.create_timer(float(g('publish_period')), self._publish_maps)
        self.create_timer(1.0, self._log_csv)
        self.create_timer(30.0, self._log_info)
        if is_ground:
            self.create_timer(float(g('pose_period')), self._send_pose)
            self.create_timer(1.0 / max(float(g('fleet_rate')), 1e-3), self._publish_fleet)
        self._publish_maps(force=True)      # пустая карта: Nav2 нужна карта для активации
        self.get_logger().info(
            f'агент связи {me} ({self.kind}): карта {self.w}x{self.h}, роботы {self.robots}, дроны {self.air_robots}')

    # ---------- радио ----------
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _send_raw(self, channel, payload):
        p = CommsPacket()
        p.stamp = self.get_clock().now().to_msg()
        p.src = self.me
        p.channel = channel
        self.tx_seq += 1
        p.seq = self.tx_seq
        p.payload = list(payload) if not isinstance(payload, (bytes, bytearray)) else payload
        self.tx.publish(p)
        self.bytes_tx[channel] = self.bytes_tx.get(channel, 0) + len(payload)
        if self.bw > 0 and channel in ('map', 'obs'):
            self.backlog_s += (len(payload) + 32) / self.bw

    def _on_status(self, m):
        if len(m.data) >= 3:
            self.backlog_s, self.bw = float(m.data[0]), float(m.data[2])

    def _busy(self):
        """Очередь данных радио полна: повторы и ретрансляция подождут."""
        return self.backlog_s > self.max_backlog_s

    def _send(self, channel, msg):
        self._send_raw(channel, serialize_message(msg))

    def _fresh_neighbors(self):
        now = self._now()
        return {n for n, t in self.neigh.items() if now - t <= self.neighbor_ttl}

    def _on_rx(self, pkt):
        if pkt.src == self.me:
            return
        self.neigh[pkt.src] = self._now()
        ch = pkt.channel
        data = bytes(pkt.payload)
        if ch == 'beacon':
            self._on_beacon(deserialize_message(data, CommsBeacon))
        elif ch == 'map':
            self._on_map_packet(deserialize_message(data, MapLayer))
        elif ch == 'obs':
            m = deserialize_message(data, ObsBatch)
            self._deliver(self.log.add(m.robot, m.seq, data))
        elif ch == self.bid_ch and self.bid_ch:
            self.pub_bid.publish(deserialize_message(data, ExplorationBid))
        elif ch == 'pose' and self.kind == 'ground':
            o = deserialize_message(data, PoseStamped)
            if pkt.src in self.robots:
                self.peer_pose[pkt.src] = (o.pose.position.x, o.pose.position.y,
                                           _yaw(o.pose.orientation), self._now())

    # ---------- маяк ----------
    def _send_beacon(self):
        b = CommsBeacon()
        b.header.stamp = self.get_clock().now().to_msg()
        b.node = self.me
        b.map_sources = list(self.layers)
        b.map_versions = [ly.have for ly in self.layers.values()]
        srcs = self.log.sources()
        b.obs_sources = srcs
        b.obs_seqs = [self.log.have(s) for s in srcs]
        b.neighbors = sorted(self._fresh_neighbors())
        self._send('beacon', b)
        self.pub_have.publish(b)    # то же локально: для оценки (coverage ждёт синхронизации базы)

    def _on_beacon(self, b):
        self.peers[b.node] = dict(
            t=self._now(),
            map=dict(zip(b.map_sources, (int(v) for v in b.map_versions))),
            obs=dict(zip(b.obs_sources, (int(v) for v in b.obs_seqs))),
            nb=set(b.neighbors))

    def _fresh_peers(self, what):
        """{узел: (have, соседи)} для узлов со свежим маяком, которые мы ещё слышим."""
        now = self._now()
        nb = self._fresh_neighbors()
        return {n: (p[what], p['nb']) for n, p in self.peers.items()
                if n in nb and now - p['t'] <= self.neighbor_ttl}

    # ---------- карта ----------
    def _on_own_layer(self, m):
        if int(m.width) * int(m.height) != self.ncell:
            return
        ly = self.layers.get(self.me)
        if ly is None:
            ly = self.layers[self.me] = ms.SourceLayer(self.ncell, m.kind)
        if m.version <= ly.have and ly.have:
            return
        ly.apply_full(ms.decode(m.data, m.encoding), m.version)

    def _on_map_packet(self, m):
        if m.source == self.me or int(m.width) * int(m.height) != self.ncell:
            return
        ly = self.layers.get(m.source)
        if ly is None:
            ly = self.layers[m.source] = ms.SourceLayer(self.ncell, m.kind)
        ly.apply_delta(ms.decode(m.data, m.encoding), int(m.base_version), int(m.version))

    def _map_tick(self):
        now = self._now()
        peers = self._fresh_peers('map')
        if not peers:
            return
        mynb = set(peers)
        # свой слой первым: при полной очереди ретрансляция чужих ждёт
        for s in sorted(self.layers, key=lambda k: (k != self.me, k)):
            ly = self.layers[s]
            if ly.have == 0:
                continue
            if self._busy():
                break
            targets = ms.serve_targets(self.me, s, ly.have, peers, mynb)
            if not targets:
                continue
            base = min(peers[p][0].get(s, 0) for p in targets)
            last = self.sent_map.get(s)
            if last and last[0] >= ly.have and last[1] <= base and now - last[2] < self.map_resend_s:
                continue
            data, _ = ly.make_delta(base)
            m = MapLayer()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'world'
            m.source = s
            m.kind = ly.kind
            m.version = ly.have
            m.base_version = base
            m.width, m.height = self.w, self.h
            m.resolution = self.res
            m.origin_x, m.origin_y = self.ox, self.oy
            m.encoding = ms.ENC_ZLIB
            m.data = data
            self._send('map', m)
            self.sent_map[s] = (ly.have, base, now)

    def _grid_msg(self, grid):
        m = OccupancyGrid()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'world'
        m.info.resolution = self.res
        m.info.width, m.info.height = self.w, self.h
        m.info.origin.position.x = self.ox
        m.info.origin.position.y = self.oy
        m.info.origin.orientation.w = 1.0
        m.data = array.array('b', grid.tobytes())
        return m

    def _publish_maps(self, force=False):
        force = force is True or self._first_pub
        if not force and not any(ly.changed for ly in self.layers.values()):
            return
        self._first_pub = False
        for ly in self.layers.values():
            ly.changed = False
        gl = [ly for ly in self.layers.values() if ly.kind == MapLayer.KIND_GROUND]
        al = [ly for ly in self.layers.values() if ly.kind == MapLayer.KIND_AIR]
        empty = np.full(self.ncell, -1, dtype=np.int8)
        gm, am = ms.merge(gl), ms.merge(al)
        g_grid = ms.to_grid(*gm) if gm else empty
        a_grid = ms.to_grid(*am) if am else empty
        if self.kind == 'air':
            self.pub_map['air'].publish(self._grid_msg(a_grid))
            return
        fused = ms.fuse(g_grid, gm[1], a_grid) if gm else a_grid
        self.pub_map['map'].publish(self._grid_msg(fused))
        if self.kind == 'base':
            self.pub_map['ground'].publish(self._grid_msg(g_grid))
            self.pub_map['air'].publish(self._grid_msg(a_grid))

    # ---------- наблюдения ----------
    def _on_det(self, d):
        self.pend_det.append(d)

    def _on_rgb(self, d):
        if d.robot == self.me and self.pose is not None:
            self.pend_rgb.append((d, *self.pose))

    def _on_pose(self, o):
        p = o.pose.pose
        self.pose = (p.position.x, p.position.y, _yaw(p.orientation))
        self.pose_msg = o
        t = o.header.stamp.sec + o.header.stamp.nanosec * 1e-9
        if t - self.last_view_t >= self.view_dt:
            self.last_view_t = t
            self.pend_view.append((t, *self.pose))

    def _deliver(self, items):
        for _, data in items:
            self.pub_obs.publish(deserialize_message(data, ObsBatch))

    def _obs_tick(self):
        now = self._now()
        if self.kind != 'base' and (self.pend_det or self.pend_rgb or self.pend_view):
            b = ObsBatch()
            b.header.stamp = self.get_clock().now().to_msg()
            b.header.frame_id = 'world'
            b.robot = self.me
            self.obs_seq += 1
            b.seq = self.obs_seq
            b.detections = self.pend_det
            b.rgb = [r[0] for r in self.pend_rgb]
            b.rgb_pose = [float(v) for r in self.pend_rgb for v in r[1:]]
            b.view_t = [float(v[0]) for v in self.pend_view]
            b.view_pose = [float(c) for v in self.pend_view for c in v[1:]]
            self.pend_det, self.pend_rgb, self.pend_view = [], [], []
            data = serialize_message(b)
            self.log.add(self.me, b.seq, data)
            self.pub_obs.publish(b)
            if self._fresh_neighbors():
                self._send_raw('obs', data)
                self.obs_sent_t[(self.me, b.seq)] = now
        # синхронизация: что нужно соседям
        peers = self._fresh_peers('obs')
        if not peers:
            return
        mynb = set(peers)
        for s in sorted(self.log.sources(), key=lambda k: (k != self.me, k)):
            have = self.log.have(s)
            targets = ms.serve_targets(self.me, s, have, peers, mynb)
            if not targets:
                continue
            lo = min(peers[p][0].get(s, 0) for p in targets)
            n = 0
            for seq, data in self.log.get_range(s, lo, have):
                if n >= self.obs_burst or self._busy():
                    break
                if now - self.obs_sent_t.get((s, seq), -1e9) < self.obs_resend_s:
                    continue
                self._send_raw('obs', data)
                self.obs_sent_t[(s, seq)] = now
                n += 1

    # ---------- ставки ----------
    def _on_bid(self, m):
        if m.robot_id == self.me:
            self._send(self.bid_ch, m)

    # ---------- позы ----------
    def _send_pose(self):
        # компактная поза (~80 Б) вместо Odometry с ковариациями (~760 Б)
        if self.pose_msg is not None:
            ps = PoseStamped()
            ps.header = self.pose_msg.header
            ps.pose = self.pose_msg.pose.pose
            self._send('pose', ps)

    def _publish_fleet(self):
        now = self._now()
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = 'world'
        for r in self.robots:
            p = Pose()
            p.orientation.w = 1.0
            pose = None
            if r == self.me:
                pose = self.pose
            elif r in self.peer_pose and now - self.peer_pose[r][3] <= self.pose_ttl:
                pose = self.peer_pose[r][:3]
            if pose is None:
                p.position.x = p.position.y = p.position.z = float('nan')
            else:
                p.position.x, p.position.y = float(pose[0]), float(pose[1])
                p.orientation.z = math.sin(pose[2] / 2)
                p.orientation.w = math.cos(pose[2] / 2)
            pa.poses.append(p)
        self.pub_fleet.publish(pa)

    # ---------- журналы ----------
    def _log_csv(self):
        if not self.csv:
            return
        t = self._now()
        nn = len(self._fresh_neighbors())
        srcs = set(self.layers) | set(self.log.sources())
        for s in sorted(srcs):
            ly = self.layers.get(s)
            self.csv.write(f'{t:.1f},{s},{ly.have if ly else 0},{self.log.have(s)},{nn}\n')
        self.csv.flush()

    def _log_info(self):
        mv = {s: ly.have for s, ly in self.layers.items()}
        ov = {s: self.log.have(s) for s in self.log.sources()}
        self.get_logger().info(
            f'связь {self.me}: соседи {sorted(self._fresh_neighbors())}, карта {mv}, наблюдения {ov}, '
            f'отправлено байт {self.bytes_tx}')


def main():
    rclpy.init()
    node = Agent()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.csv:
            node.csv.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

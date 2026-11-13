"""Узел comms_sim: радиоканал роя. Все пакеты роботов идут через него.

Модель: дальность, стены (линия видимости), пропускная способность с очередями,
потери фрагментов с повторами, задержка с джиттером, отключения узлов.
Подробности и пресеты: rs_comms/link_model.py.
"""
import csv
import heapq
import json
import math
import os
import random
from collections import deque

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Float32MultiArray

from rs_interfaces.msg import CommsPacket
from rs_comms import link_model as lm

CONTROL = {'bid', 'air_bid', 'pose', 'beacon'}
HEADER_BYTES = 32
STAT_KEYS = ['tx_pkts', 'tx_bytes', 'qdrop_pkts', 'qdrop_bytes', 'rx_pkts',
             'rx_bytes', 'lost_pkts', 'nolink_pkts']


class Radio:
    """Радио одного узла: две очереди (управление первой, потом данные)."""

    def __init__(self):
        self.q = {'control': deque(), 'bulk': deque()}
        self.qbytes = {'control': 0, 'bulk': 0}
        self.cur = None      # (msg, size, t_end)


class CommsSim(Node):
    def __init__(self):
        super().__init__('comms_sim')
        sl = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('nodes', ['base'], sl)
        self.declare_parameter('robots', [], sl)
        self.declare_parameter('air_robots', [], sl)
        self.declare_parameter('gt_file', '')
        self.declare_parameter('base_xyz', [0.0, 0.0, 2.5])
        self.declare_parameter('level', 'good')
        for k in lm.PARAM_KEYS:
            self.declare_parameter(k, -1.0)
        self.declare_parameter('outages', '', sl)
        self.declare_parameter('antenna_h', 0.3)
        self.declare_parameter('out_dir', '')
        self.declare_parameter('seed', 1)
        self.declare_parameter('link_period', 0.2)
        self.declare_parameter('tick', 0.01)
        self.declare_parameter('marker_rate', 2.0)

        gp = self.get_parameter
        self.nodes = list(gp('nodes').value or [])
        self.robots = list(gp('robots').value or [])
        if not self.robots:
            raise RuntimeError('параметр robots пуст: список роботов задаёт launch (explore.launch.py), запасного списка нет')
        self.air_robots = list(gp('air_robots').value or [])
        self.base_xyz = [float(v) for v in gp('base_xyz').value]
        self.level = gp('level').value
        self.P = lm.resolve_params(
            self.level, {k: gp(k).value for k in lm.PARAM_KEYS})
        self.outages = lm.parse_outages(gp('outages').value)
        self.antenna_h = float(gp('antenna_h').value)
        self.out_dir = gp('out_dir').value
        self.rng = random.Random(int(gp('seed').value))
        self.unlimited_bw = not self.P.bandwidth_Bps
        self.ideal = math.isinf(self.P.range_m)

        gt = {}
        gt_file = gp('gt_file').value
        if gt_file:
            with open(gt_file) as f:
                gt = yaml.safe_load(f) or {}
        self.walls = lm.load_walls(gt)

        # позиции: узлы без позиции связи не имеют (кроме base)
        self.pos = {}
        self.is_air = set(self.air_robots)
        if 'base' in self.nodes:
            self.pos['base'] = tuple(self.base_xyz)
        for r in self.robots + self.air_robots:
            self.create_subscription(
                Odometry, '/%s/gt_odom' % r,
                lambda m, r=r: self._on_odom(r, m), 10)

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=200)
        self.rx_pub = {n: self.create_publisher(
            CommsPacket, '/%s/comms/rx' % n, qos) for n in self.nodes}
        self.create_subscription(CommsPacket, '/comms/tx', self._on_tx, qos)
        self.mk_pub = self.create_publisher(MarkerArray, '/comms/markers', 10)
        # состояние своего радио для агента (как очередь сетевого интерфейса): [секунд эфира в очереди
        # данных, секунд эфира в очереди управления, полоса Б/с (0 = без ограничения)]
        self.st_pub = {n: self.create_publisher(
            Float32MultiArray, '/%s/comms/status' % n, 5) for n in self.nodes}

        self.radio = {n: Radio() for n in self.nodes}
        self.stats = {n: {k: 0 for k in STAT_KEYS} for n in self.nodes}
        self.channels = {}
        self.links = {}      # (a, b), a < b -> (up, d, walls, d_eff, p_frag)
        self.heap = []
        self.hcount = 0
        self.warned = set()
        self.tx_total = 0

        self._open_files()
        self._update_links()
        self.next_rec = None
        self.next_log = None
        self.create_timer(float(gp('link_period').value), self._update_links)
        self.create_timer(float(gp('tick').value), self._tick)
        self.create_timer(0.1, self._publish_status)
        self.create_timer(1.0 / max(float(gp('marker_rate').value), 0.1),
                          self._publish_markers)
        self.get_logger().info(
            'comms_sim: узлов %d, уровень %s, стен %d, отключений %d'
            % (len(self.nodes), self.level, len(self.walls), len(self.outages)))

    # ---------- файлы ----------
    def _open_files(self):
        self.files = {}
        if not self.out_dir:
            return
        d = os.path.join(self.out_dir, 'comms')
        os.makedirs(d, exist_ok=True)
        params = {k: (None if isinstance(v, float) and math.isinf(v) else v)
                  for k, v in self.P.items()}
        with open(os.path.join(d, 'comms_params.json'), 'w') as f:
            json.dump({'level': self.level, 'params': params,
                       'outages': self.outages, 'base_xyz': self.base_xyz,
                       'nodes': self.nodes}, f, indent=1)
        heads = {
            'links.csv': 't,a,b,up,d,walls,d_eff,p_frag',
            'traffic.csv': 't,node,' + ','.join(STAT_KEYS) + ',queue_bytes',
            'channels.csv': 't,channel,tx_pkts,tx_bytes'}
        for name, head in heads.items():
            f = open(os.path.join(d, name), 'w')
            f.write(head + '\n')
            f.flush()
            self.files[name] = f

    def _write_rows(self, name, rows):
        f = self.files.get(name)
        if f is None:
            return
        w = csv.writer(f, lineterminator='\n')
        for r in rows:
            w.writerow([round(v, 3) if isinstance(v, float) else v for v in r])
        f.flush()

    # ---------- время ----------
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------- позиции и связи ----------
    def _on_odom(self, r, m):
        p = m.pose.pose.position
        z = p.z if r in self.is_air else p.z + self.antenna_h
        self.pos[r] = (p.x, p.y, z)

    def _update_links(self):
        t = self._now()
        out = {}
        ns = sorted(self.nodes)
        for i, a in enumerate(ns):
            for b in ns[i + 1:]:
                if self.ideal:
                    # идеальная связь: всегда есть, кроме заданных отключений
                    pa, pb = self.pos.get(a), self.pos.get(b)
                    d = math.dist(pa, pb) if pa and pb else 0.0
                    up = not (lm.in_outage(self.outages, a, t) or lm.in_outage(self.outages, b, t))
                    out[(a, b)] = (up, d, 0, d, self.P.loss_base)
                    continue
                pa, pb = self.pos.get(a), self.pos.get(b)
                if (pa is None or pb is None
                        or lm.in_outage(self.outages, a, t)
                        or lm.in_outage(self.outages, b, t)):
                    out[(a, b)] = (False, 0.0, 0, 0.0, 0.0)
                    continue
                d = math.dist(pa, pb)
                nw = lm.count_walls(pa, pb, self.walls)
                up, d_eff, pf = lm.link_quality(d, nw, self.P)
                out[(a, b)] = (up, d, nw, d_eff, pf)
        self.links = out

    def _link(self, a, b):
        return self.links.get((a, b) if a < b else (b, a))

    def _usable(self, a, b, t):
        """Связь a-b сейчас (с проверкой отключений на момент t)."""
        l = self._link(a, b)
        if l is None or not l[0]:
            return None
        if (lm.in_outage(self.outages, a, t)
                or lm.in_outage(self.outages, b, t)):
            return None
        return l

    # ---------- приём пакетов от узлов ----------
    def _on_tx(self, msg):
        src = msg.src
        if src not in self.radio:
            if src not in self.warned:
                self.warned.add(src)
                self.get_logger().warn('пакет от неизвестного узла %r: игнорирую' % src)
            return
        t = self._now()
        size = len(msg.payload) + HEADER_BYTES
        st = self.stats[src]
        st['tx_pkts'] += 1
        st['tx_bytes'] += size
        ch = self.channels.setdefault(msg.channel, [0, 0])
        ch[0] += 1
        ch[1] += size
        if self.unlimited_bw:
            self._finish(src, msg, size, t)
            return
        cls = 'control' if msg.channel in CONTROL else 'bulk'
        r = self.radio[src]
        if (r.qbytes[cls] + size) / self.P.bandwidth_Bps > self.P.queue_s:
            st['qdrop_pkts'] += 1
            st['qdrop_bytes'] += size
            return
        r.q[cls].append((msg, size))
        r.qbytes[cls] += size

    def _tick(self):
        t = self._now()
        if not self.unlimited_bw:
            for n, r in self.radio.items():
                while True:
                    if r.cur is not None:
                        if r.cur[2] > t:
                            break
                        msg, size, t_end = r.cur
                        r.cur = None
                        self._finish(n, msg, size, t_end)
                    for cls in ('control', 'bulk'):
                        if r.q[cls]:
                            msg, size = r.q[cls].popleft()
                            r.qbytes[cls] -= size
                            r.cur = (msg, size, max(t, 0.0)
                                     + size / self.P.bandwidth_Bps)
                            break
                    else:
                        break
                    # новая передача началась; если уже закончилась (малый размер
                    # при большом tick), следующий оборот цикла её завершит
                    if r.cur[2] > t:
                        break
        while self.heap and self.heap[0][0] <= t:
            _, _, node, msg = heapq.heappop(self.heap)
            self._deliver(node, msg)
        if self.next_rec is None:
            self.next_rec = t + 1.0
            self.next_log = t + 30.0
        if t >= self.next_rec:
            self.next_rec = t + 1.0
            self._record(t)
        if t >= self.next_log:
            self.next_log = t + 30.0
            self._log_summary()

    def _publish_status(self):
        bw = 0.0 if self.unlimited_bw else float(self.P.bandwidth_Bps)
        for n, r in self.radio.items():
            m = Float32MultiArray()
            if bw > 0:
                cur = r.cur[1] if r.cur is not None else 0
                m.data = [(r.qbytes['bulk'] + cur) / bw, r.qbytes['control'] / bw, bw]
            else:
                m.data = [0.0, 0.0, 0.0]
            self.st_pub[n].publish(m)

    def _finish(self, src, msg, size, t_end):
        """Передача закончилась: раздаём пакет всем, до кого дотягивается канал."""
        P = self.P
        any_link = False
        for b in self.nodes:
            if b == src:
                continue
            l = self._usable(src, b, t_end)
            if l is None:
                continue
            any_link = True
            if self.rng.random() < lm.packet_success_prob(size, l[4], P):
                delay = P.delay_s + (self.rng.uniform(0.0, P.jitter_s)
                                     if P.jitter_s > 0 else 0.0)
                if delay <= 0.0:
                    self._deliver(b, msg)
                else:
                    self.hcount += 1
                    heapq.heappush(self.heap, (t_end + delay, self.hcount, b, msg))
            else:
                self.stats[b]['lost_pkts'] += 1
        if not any_link:
            self.stats[src]['nolink_pkts'] += 1

    def _deliver(self, node, msg):
        st = self.stats[node]
        st['rx_pkts'] += 1
        st['rx_bytes'] += len(msg.payload) + HEADER_BYTES
        self.rx_pub[node].publish(msg)

    # ---------- запись, логи, маркеры ----------
    def _record(self, t):
        rows = [(t, a, b, int(l[0]), l[1], l[2], l[3], l[4])
                for (a, b), l in sorted(self.links.items())]
        self._write_rows('links.csv', rows)
        rows = []
        for n in sorted(self.nodes):
            s = self.stats[n]
            qb = sum(self.radio[n].qbytes.values())
            rows.append((t, n) + tuple(s[k] for k in STAT_KEYS) + (qb,))
        self._write_rows('traffic.csv', rows)
        self._write_rows('channels.csv', [(t, c, v[0], v[1])
                                          for c, v in sorted(self.channels.items())])

    def _log_summary(self):
        up = sum(1 for l in self.links.values() if l[0])
        tot = sum(s['tx_bytes'] for s in self.stats.values())
        qd = sum(s['qdrop_pkts'] for s in self.stats.values())
        lost = sum(s['lost_pkts'] for s in self.stats.values())
        self.get_logger().info(
            'связь: каналов %d из %d, отправлено %d Б, потеряно в очереди %d, потеряно в эфире %d'
            % (up, len(self.links), tot, qd, lost))

    def _publish_markers(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = now
        m.ns, m.id = 'links', 0
        m.type, m.action = Marker.LINE_LIST, Marker.ADD
        m.scale.x = 0.06
        m.pose.orientation.w = 1.0
        for (a, b), l in self.links.items():
            if not l[0]:
                continue
            pa, pb = self.pos.get(a), self.pos.get(b)
            if pa is None or pb is None:
                continue
            col = (ColorRGBA(r=0.1, g=0.9, b=0.1, a=1.0) if l[4] < 0.1 else
                   ColorRGBA(r=0.95, g=0.85, b=0.1, a=1.0) if l[4] < 0.3 else
                   ColorRGBA(r=0.95, g=0.1, b=0.1, a=1.0))
            for p in (pa, pb):
                m.points.append(Point(x=p[0], y=p[1], z=p[2]))
                m.colors.append(col)
        ma.markers.append(m)
        if 'base' in self.pos:
            b = Marker()
            b.header.frame_id = 'world'
            b.header.stamp = now
            b.ns, b.id = 'base', 0
            b.type, b.action = Marker.CUBE, Marker.ADD
            bx, by, bz = self.pos['base']
            b.pose.position.x, b.pose.position.y, b.pose.position.z = bx, by, bz
            b.pose.orientation.w = 1.0
            b.scale.x = b.scale.y = 0.4
            b.scale.z = 0.4
            b.color = ColorRGBA(r=0.2, g=0.4, b=1.0, a=1.0)
            ma.markers.append(b)
        self.mk_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = CommsSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for f in node.files.values():
            f.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

"""Узел people_fusion: детекции всех роботов -> отметки людей (CANDIDATE/CONFIRMED/REJECTED).

Публикует /people/marks (1 Гц), /people/markers (MarkerArray), /people/inspect_requests
(кандидаты без достаточного числа близких наблюдений). Пишет people.json в out_dir.
"""
import json
import math
import os

import rclpy
import numpy as np
from geometry_msgs.msg import Point, PoseArray
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rs_interfaces.msg import PersonMark, PersonMarkArray, ThermalDetection
from visualization_msgs.msg import Marker, MarkerArray

from rs_perception import marks as M

COLORS = {M.CANDIDATE: (1.0, 0.9, 0.1), M.CONFIRMED: (0.1, 0.9, 0.2), M.REJECTED: (0.55, 0.55, 0.55)}


def _t(sec):
    s = int(sec)
    return s, int((sec - s) * 1e9)


class PeopleFusion(Node):
    def __init__(self):
        super().__init__('people_fusion')
        for k, v in M.DEFAULTS.items():
            self.declare_parameter(k, v)
        self.declare_parameter('out_dir', '')
        self.declare_parameter('robots', ['ground_1', 'ground_2', 'ground_3'])
        params = {k: self.get_parameter(k).value for k in M.DEFAULTS}
        self.fusion = M.Fusion(params)
        self.out_dir = self.get_parameter('out_dir').value
        if self.out_dir:
            os.makedirs(self.out_dir, exist_ok=True)
        self.map = None
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.robots = list(self.get_parameter('robots').value)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(PoseArray, '/fleet/poses', self._on_poses, 5)
        self.create_subscription(ThermalDetection, '/perception/detections', self._on_det, 50)
        self.pub_marks = self.create_publisher(PersonMarkArray, '/people/marks', 5)
        self.pub_markers = self.create_publisher(MarkerArray, '/people/markers', 5)
        self.pub_insp = self.create_publisher(PersonMarkArray, '/people/inspect_requests', 5)
        self.create_timer(1.0, self._tick)
        self.n_det = 0
        self.ticks = 0
        self.det_log = open(os.path.join(self.out_dir, 'detections.jsonl'), 'w') if self.out_dir else None

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_map(self, msg):
        h, w = msg.info.height, msg.info.width
        self.map = (np.asarray(msg.data, dtype=np.int8).reshape(h, w), msg.info.resolution,
                    msg.info.origin.position.x, msg.info.origin.position.y)

    def _los(self, x0, y0, x1, y1):
        """Прямая видимость на карте: нет занятых клеток на отрезке (у цели 0.45 м не считаем: объект сам может быть в карте)."""
        if self.map is None:
            return True
        data, res, ox, oy = self.map
        n = int(max(abs(x1 - x0), abs(y1 - y0)) / res) + 2
        xs = np.linspace(x0, x1, n)
        ys = np.linspace(y0, y1, n)
        i = np.floor((xs - ox) / res).astype(int)
        j = np.floor((ys - oy) / res).astype(int)
        h, w = data.shape
        if ((i < 0) | (i >= w) | (j < 0) | (j >= h)).any():
            return False
        occ = data[j, i] >= 65
        occ &= np.hypot(xs - x1, ys - y1) > 0.45
        return not occ.any()

    def _on_poses(self, msg):
        t = self._now()
        for rid, p in zip(self.robots, msg.poses):
            if not math.isfinite(p.position.x):
                continue
            yaw = math.atan2(2 * p.orientation.z * p.orientation.w, 1 - 2 * p.orientation.z ** 2)
            for m in self.fusion.observe_view(rid, p.position.x, p.position.y, yaw, t, self._los):
                self.get_logger().info(
                    f'{m.id} -> REJECTED (not_seen_close) at ({m.x:.2f}, {m.y:.2f}) n={m.n_obs} close={m.n_close}')
                self._write_json()

    def _on_det(self, d):
        t = d.header.stamp.sec + d.header.stamp.nanosec * 1e-9
        m, changed = self.fusion.update(d.position.x, d.position.y, d.robot, d.range_m,
                                        d.mean_temp_k, d.height_m, d.length_m, t)
        self.n_det += 1
        if self.det_log:
            nz = lambda v: None if math.isnan(v) else round(float(v), 3)  # noqa: E731
            self.det_log.write(json.dumps(dict(
                t=round(t, 2), robot=d.robot, x=round(d.position.x, 3), y=round(d.position.y, 3),
                range=round(d.range_m, 2), temp=round(d.mean_temp_k, 2), max_temp=round(d.max_temp_k, 2),
                px=d.blob_px, h=nz(d.height_m), len=nz(d.length_m), n_pts=d.n_lidar_pts, mark=m.id)) + '\n')
            if self.n_det % 20 == 0:
                self.det_log.flush()
        if changed:
            self.get_logger().info(
                f'{m.id} -> {M.NAMES[m.status]} ({m.reason}) at ({m.x:.2f}, {m.y:.2f}) n={m.n_obs} close={m.n_close} '
                f'T={m.mean_temp():.1f} h={m.height():.2f} len={m.length():.2f}')
            self._write_json()

    def _msg(self, m):
        pm = PersonMark()
        pm.id = m.id
        pm.status = m.status
        pm.position = Point(x=m.x, y=m.y, z=0.0)
        pm.confidence = float(self.fusion.confidence(m))
        pm.n_obs = m.n_obs
        pm.robots = list(m.robots)
        pm.min_range_m = float(m.min_range)
        pm.mean_temp_k = float(m.mean_temp())
        pm.height_m = float(m.height())
        pm.length_m = float(m.length())
        pm.first_seen.sec, pm.first_seen.nanosec = _t(m.first_seen)
        pm.status_time.sec, pm.status_time.nanosec = _t(m.status_time)
        return pm

    def _tick(self):
        self.ticks += 1
        if self.ticks % 10 == 0:
            self._write_json()
        now = self.get_clock().now().to_msg()
        arr = PersonMarkArray()
        arr.header.stamp = now
        arr.header.frame_id = 'world'
        arr.marks = [self._msg(m) for m in self.fusion.reported()]
        self.pub_marks.publish(arr)
        insp = PersonMarkArray()
        insp.header = arr.header
        insp.marks = [self._msg(m) for m in self.fusion.needs_inspection()]
        self.pub_insp.publish(insp)
        mk = MarkerArray()
        for m in self.fusion.reported():
            pm = self._msg(m)
            r, g, b = COLORS[m.status]
            s = Marker()
            s.header = arr.header
            s.ns, s.id = 'mark', int(m.id.split('_')[1])
            s.type, s.action = Marker.SPHERE, Marker.ADD
            s.pose.position = Point(x=m.x, y=m.y, z=0.3)
            s.pose.orientation.w = 1.0
            s.scale.x = s.scale.y = s.scale.z = 0.4
            s.color.r, s.color.g, s.color.b, s.color.a = r, g, b, 0.9
            mk.markers.append(s)
            tx = Marker()
            tx.header = arr.header
            tx.ns, tx.id = 'label', s.id
            tx.type, tx.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            tx.pose.position = Point(x=m.x, y=m.y, z=0.9)
            tx.pose.orientation.w = 1.0
            tx.scale.z = 0.35
            tx.color.r = tx.color.g = tx.color.b = tx.color.a = 1.0
            tx.text = f'{m.id} {M.NAMES[m.status][:4]} {pm.confidence:.2f}'
            mk.markers.append(tx)
        for rid in self.fusion.removed:
            for ns in ('mark', 'label'):
                dm = Marker()
                dm.header = arr.header
                dm.ns, dm.id, dm.action = ns, int(rid.split('_')[1]), Marker.DELETE
                mk.markers.append(dm)
        self.fusion.removed = []
        self.pub_markers.publish(mk)

    def _write_json(self):
        if not self.out_dir:
            return
        out = dict(sim_time=self._now(), marks=[])
        for m in self.fusion.reported():
            pm = self._msg(m)
            nz = lambda v: None if math.isnan(v) else round(float(v), 3)  # noqa: E731
            out['marks'].append(dict(
                id=m.id, status=M.NAMES[m.status], x=round(m.x, 3), y=round(m.y, 3),
                confidence=round(pm.confidence, 3), n_obs=m.n_obs, n_close=m.n_close,
                robots=list(m.robots), min_range_m=round(m.min_range, 3), mean_temp_k=nz(m.mean_temp()),
                height_m=nz(m.height()), length_m=nz(m.length()), first_seen=round(m.first_seen, 2),
                status_time=round(m.status_time, 2),
                reason=m.reason,
                history=[dict(t=round(t, 2), status=s, reason=r) for t, s, r in m.history]))
        tmp = os.path.join(self.out_dir, 'people.json.tmp')
        with open(tmp, 'w') as f:
            json.dump(out, f, indent=1)
        os.replace(tmp, os.path.join(self.out_dir, 'people.json'))


def main():
    rclpy.init()
    node = PeopleFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._write_json()
        if node.det_log:
            node.det_log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

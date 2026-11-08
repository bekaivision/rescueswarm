"""Узел thermal_detector (по одному на робота): пятна на тепловом кадре -> ThermalDetection.

Подписки (в пространстве имён робота): thermal/image_raw (mono16, 0.01 K на отсчёт),
thermal/camera_info, lidar/points. Публикует /perception/detections (кадр world).
Дальность: точки лидара внутри рамки пятна (ближайший кластер), иначе пересечение луча с полом.
"""
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rs_interfaces.msg import ThermalDetection
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

from rs_perception import blobs
from rs_perception.geometry import apply, invert, tf_matrix

NAN = float('nan')


class ThermalDetector(Node):
    def __init__(self):
        super().__init__('thermal_detector')
        P = self.declare_parameter
        P('robot', 'ground_1')
        P('hot_k', 303.0)
        P('min_blob_px', 4)
        P('max_range_m', 12.0)
        P('temp_scale', 0.01)          # K на отсчёт
        P('cluster_m', 1.5)            # глубина кластера лидара от ближайшей точки
        P('min_point_z', 0.04)         # точки ниже считаются полом
        P('center_offset_m', 0.2)      # сдвиг от видимой грани к центру объекта
        P('min_shape_pts', 5)          # меньше точек лидара: длина не считается (NaN)
        P('max_cloud_age_s', 0.2)
        P('tf_timeout_s', 0.1)
        P('hfov', 1.047)               # если нет camera_info
        P('floor_contact_height', True)   # пятно не касается пола: высоте по лидару не верим
        P('debug_dir', '')             # куда сложить один кадр с рамками (пусто = не писать)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.g = g
        self.robot = g('robot')
        self.frame = f'{self.robot}/thermal_link'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.K = None
        self.clouds = deque(maxlen=6)    # (stamp_s, frame_id, ndarray (N,3))
        self.debug_saved = 0
        self.debug_best = 40
        self.n_img = self.n_det = self.n_drop = self.n_incons = self.n_nofloor = 0
        self.create_subscription(CameraInfo, 'thermal/camera_info', self._on_info, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, 'lidar/points', self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(Image, 'thermal/image_raw', self._on_image, qos_profile_sensor_data)
        self.pub = self.create_publisher(ThermalDetection, '/perception/detections', 20)
        self.create_timer(30.0, self._stats)

    def _stats(self):
        self.get_logger().info(f'кадров {self.n_img}, детекций {self.n_det}, отброшено {self.n_drop}, лидар не согласован с полом {self.n_incons}, '
                               f'без касания пола {self.n_nofloor}')

    def _on_info(self, m):
        if self.K is None and m.k[0] > 0:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def _on_cloud(self, m):
        try:
            pts = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'облако: {e}', throttle_duration_sec=10.0)
            return
        pts = pts.astype(np.float64)
        pts = pts[np.isfinite(pts).all(axis=1)]
        self.clouds.append((Time.from_msg(m.header.stamp).nanoseconds * 1e-9, m.header.frame_id, pts))

    def _lookup(self, target, source, stamp):
        return tf_matrix(self.tf_buffer.lookup_transform(
            target, source, stamp, timeout=Duration(seconds=float(self.g('tf_timeout_s')))))

    def _on_image(self, m):
        self.n_img += 1
        h, w = m.height, m.width
        dt = '>u2' if m.is_bigendian else '<u2'
        raw = np.frombuffer(m.data, dtype=dt).reshape(h, m.step // 2)[:, :w]
        temp = raw.astype(np.float64) * float(self.g('temp_scale'))
        bl = blobs.find_blobs(temp, float(self.g('hot_k')), int(self.g('min_blob_px')))
        score = sum(b['px'] for b in bl if b['px'] <= 600) if bl else 0   # кадр с несколькими обычными пятнами
        if self.g('debug_dir') and len(bl) >= 2 and score > self.debug_best:
            self.debug_best = score
            self._save_debug(temp, bl, m)
        if not bl:
            return
        if self.K is not None:
            fx, fy, cx, cy = self.K
        else:
            fx = fy = (w / 2.0) / math.tan(float(self.g('hfov')) / 2.0)
            cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
        stamp = Time.from_msg(m.header.stamp)
        try:
            T_wt = self._lookup('world', self.frame, stamp)
        except TransformException as e:
            self.n_drop += 1
            self.get_logger().warn(f'TF world<-{self.frame}: {e}', throttle_duration_sec=10.0)
            return
        t_img = stamp.nanoseconds * 1e-9
        pts_t = pts_z = None
        best = None
        for c in self.clouds:
            if abs(c[0] - t_img) <= float(self.g('max_cloud_age_s')) and (best is None or abs(c[0] - t_img) < abs(best[0] - t_img)):
                best = c
        if best is not None and len(best[2]):
            try:
                T_wl = self._lookup('world', best[1], Time(nanoseconds=int(best[0] * 1e9)))
                pw = apply(T_wl, best[2])
                pts_z = pw[:, 2]
                pts_t = apply(invert(T_wt), pw)
            except TransformException:
                pts_t = None
        cam_z = T_wt[2, 3]
        maxr = float(self.g('max_range_m'))
        for b in bl:
            li = None
            if pts_t is not None:
                li = blobs.lidar_on_blob(pts_t, b, fx, fy, cx, cy, float(self.g('cluster_m')),
                                         float(self.g('min_point_z')), pts_z,
                                         float(self.g('center_offset_m')))
            g_rng = blobs.ground_range(b, fx, fy, cx, cy, cam_z, 1e9)
            # низ пятна упёрся в нижний край кадра: оценка по полу занижена по смыслу, не сверяем
            if li is not None and b['v1'] < h - 2 and not blobs.lidar_consistent(li['depth'], g_rng, fy, cam_z):
                self.n_incons += 1
                li = None
            if li is not None:
                rng = li['depth']
            else:
                rng = g_rng if (g_rng is not None and g_rng <= maxr) else None
            if rng is None or rng > maxr:
                continue
            if li is not None:
                p_t = np.array([[li['pos'][0], li['pos'][1], 0.0]])
            else:
                az = math.atan2(-(b['uc'] - cx), fx)    # влево положительно
                r_c = rng + float(self.g('center_offset_m'))
                p_t = np.array([[r_c * math.cos(az), r_c * math.sin(az), 0.0]])
            pw = apply(T_wt, p_t)[0]
            # проекция ширины пятна на плоскость, перпендикулярную лучу: нижняя оценка длины тела
            width = (b['u1'] - b['u0'] + 1) * rng / fx
            d = ThermalDetection()
            d.header.stamp = m.header.stamp
            d.header.frame_id = 'world'
            d.robot = self.robot
            d.position = Point(x=float(pw[0]), y=float(pw[1]), z=0.0)
            d.range_m = float(rng)
            d.mean_temp_k = b['mean_k']
            d.max_temp_k = b['max_k']
            d.blob_px = int(b['px'])
            # Низ пятна на горизонте или выше (g_rng None): тёплое видно только над препятствием
            # (камера ~0.22 м, низкий обломок той же высоты). Точки лидара тогда принадлежат
            # препятствию, их высота не высота тёплого объекта: форму не знаем (NaN), детекцию оставляем.
            no_floor = (li is not None and g_rng is None and b['v1'] < h - 2
                        and bool(self.g('floor_contact_height')))
            if no_floor:
                self.n_nofloor += 1
            d.height_m = li['height'] if (li and not no_floor) else NAN
            ll = li['length'] if (li and li['n'] >= int(self.g('min_shape_pts'))) else 0.0
            d.length_m = float(max(ll, width))
            d.n_lidar_pts = li['n'] if li else 0
            d.lidar_pad = li['pad'] if li else 255
            d.blob_w_px = int(b['u1'] - b['u0'] + 1)
            d.blob_h_px = int(b['v1'] - b['v0'] + 1)
            self.pub.publish(d)
            self.n_det += 1

    def _save_debug(self, temp, bl, m):
        import os
        os.makedirs(self.g('debug_dir'), exist_ok=True)
        lo, hi = 280.0, max(320.0, float(temp.max()))
        g8 = np.clip((temp - lo) / (hi - lo) * 255, 0, 255).astype(np.uint8)
        rgb = np.stack([g8] * 3, axis=-1)
        for b in bl:
            rgb[b['v0'], b['u0']:b['u1'] + 1] = (255, 0, 0)
            rgb[b['v1'], b['u0']:b['u1'] + 1] = (255, 0, 0)
            rgb[b['v0']:b['v1'] + 1, b['u0']] = (255, 0, 0)
            rgb[b['v0']:b['v1'] + 1, b['u1']] = (255, 0, 0)
        d = self.g('debug_dir')
        with open(os.path.join(d, f'{self.robot}_thermal_raw.pgm'), 'wb') as f:
            f.write(f'P5\n{temp.shape[1]} {temp.shape[0]}\n255\n'.encode() + g8.tobytes())
        with open(os.path.join(d, f'{self.robot}_thermal_boxes.ppm'), 'wb') as f:
            f.write(f'P6\n{temp.shape[1]} {temp.shape[0]}\n255\n'.encode() + rgb.tobytes())
        try:
            from PIL import Image as PI
            PI.fromarray(rgb).resize((temp.shape[1] * 4, temp.shape[0] * 4), PI.NEAREST).save(
                os.path.join(d, f'{self.robot}_thermal_boxes.png'))
        except Exception:  # noqa: BLE001
            pass
        np.save(os.path.join(d, f'{self.robot}_thermal_K.npy'), temp.astype(np.float32))
        self.debug_saved += 1
        self.get_logger().info(f'отладочный кадр: {d}, blobs={[(round(b["mean_k"], 2), b["px"]) for b in bl]}')


def main():
    rclpy.init()
    node = ThermalDetector()
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

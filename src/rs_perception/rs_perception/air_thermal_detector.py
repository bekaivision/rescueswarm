"""Узел air_thermal_detector (по одному на дрона): пятна на кадре надирного тепловизора -> ThermalDetection.

Запускается в пространстве имён дрона (/drone_i), подписки: thermal/image_raw (mono16, 0.01 K на отсчёт),
thermal/camera_info. Положение: луч через центр пятна (внутренние параметры + полная поза камеры из TF
на момент кадра) до пола z = 0 в world. Пятна у края кадра пропускаем. Высота и длина неизвестны (NaN):
с дрона они не измеряются, подтверждает человека только наземный робот.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rs_interfaces.msg import ThermalDetection
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from rs_perception import blobs
from rs_perception.air_geometry import pixel_floor_hit, touches_border
from rs_perception.geometry import tf_matrix

NAN = float('nan')


class AirThermalDetector(Node):
    def __init__(self):
        super().__init__('air_thermal_detector')
        P = self.declare_parameter
        P('robot', 'drone_1')
        P('hot_k', 303.0)
        P('min_blob_px', 4)
        P('max_range_m', 15.0)
        P('temp_scale', 0.01)          # K на отсчёт
        P('tf_timeout_s', 0.1)
        P('hfov', 1.396)               # если нет camera_info
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.g = g
        self.robot = g('robot')
        self.frame = f'{self.robot}/thermal_link'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.K = None
        self.n_img = self.n_det = self.n_drop = self.n_border = self.n_miss = 0
        self.create_subscription(CameraInfo, 'thermal/camera_info', self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, 'thermal/image_raw', self._on_image, qos_profile_sensor_data)
        self.pub = self.create_publisher(ThermalDetection, '/perception/detections', 20)
        self.create_timer(30.0, self._stats)

    def _stats(self):
        self.get_logger().info(f'кадров {self.n_img}, детекций {self.n_det}, у края {self.n_border}, '
                               f'луч мимо пола {self.n_miss}, без TF {self.n_drop}')

    def _on_info(self, m):
        if self.K is None and m.k[0] > 0:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def _on_image(self, m):
        self.n_img += 1
        h, w = m.height, m.width
        dt = '>u2' if m.is_bigendian else '<u2'
        raw = np.frombuffer(m.data, dtype=dt).reshape(h, m.step // 2)[:, :w]
        temp = raw.astype(np.float64) * float(self.g('temp_scale'))
        bl = blobs.find_blobs(temp, float(self.g('hot_k')), int(self.g('min_blob_px')))
        if not bl:
            return
        if self.K is not None:
            K = self.K
        else:
            f = (w / 2.0) / math.tan(float(self.g('hfov')) / 2.0)
            K = (f, f, (w - 1) / 2.0, (h - 1) / 2.0)
        stamp = Time.from_msg(m.header.stamp)
        try:
            T_wc = tf_matrix(self.tf_buffer.lookup_transform(
                'world', self.frame, stamp, timeout=Duration(seconds=float(self.g('tf_timeout_s')))))
        except TransformException as e:
            self.n_drop += 1
            self.get_logger().warn(f'TF world<-{self.frame}: {e}', throttle_duration_sec=10.0)
            return
        for b in bl:
            if touches_border(b, w, h):
                self.n_border += 1
                continue
            hit = pixel_floor_hit(b['uc'], b['vc'], K, T_wc)
            if hit is None or hit[1] > float(self.g('max_range_m')):
                self.n_miss += 1
                continue
            pw, rng = hit
            d = ThermalDetection()
            d.header.stamp = m.header.stamp
            d.header.frame_id = 'world'
            d.robot = self.robot
            d.position = Point(x=float(pw[0]), y=float(pw[1]), z=0.0)
            d.range_m = float(rng)
            d.mean_temp_k = b['mean_k']
            d.max_temp_k = b['max_k']
            d.blob_px = int(b['px'])
            d.height_m = NAN
            d.length_m = NAN
            d.n_lidar_pts = 0
            d.lidar_pad = 255
            d.blob_w_px = int(b['u1'] - b['u0'] + 1)
            d.blob_h_px = int(b['v1'] - b['v0'] + 1)
            self.pub.publish(d)
            self.n_det += 1


def main():
    rclpy.init()
    node = AirThermalDetector()
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

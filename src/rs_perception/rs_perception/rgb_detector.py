"""Узел rgb_detector: YOLO по RGB-кадрам всех роботов -> RgbDetection (дополнительный признак к тепловизору).

Подписки: /<robot>/rgb/image_raw (rgb8). Публикует /perception/rgb_detections.
Модель одна на всех роботов; кадры не копятся: хранится последний кадр робота, инференс по таймеру по кругу.
"""
import math
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rs_interfaces.msg import RgbDetection
from sensor_msgs.msg import Image


def box_angles(x0, x1, w, hfov):
    """Пиксели -> (угол центра, половина ширины); плюс = влево (против часовой)."""
    fx = (w / 2.0) / math.tan(hfov / 2.0)
    a0 = math.atan((w / 2.0 - x0) / fx)
    a1 = math.atan((w / 2.0 - x1) / fx)
    return math.atan((w / 2.0 - (x0 + x1) / 2.0) / fx), abs(a0 - a1) / 2.0


class RgbDetector(Node):
    def __init__(self):
        super().__init__('rgb_detector')
        P = self.declare_parameter
        P('robots', [], ParameterDescriptor(dynamic_typing=True))   # обязателен: список наземных роботов из launch
        P('model', '/opt/yolo/yolo11n.pt')
        P('imgsz', 640)
        P('conf', 0.10)
        P('device', 'cuda')
        P('rate_hz', 15.0)
        P('hfov', 1.047)
        P('enabled', True)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.hfov = float(g('hfov'))
        self.imgsz, self.conf = int(g('imgsz')), float(g('conf'))
        self.robots = list(g('robots') or [])
        if not self.robots:
            raise ValueError('rgb_detector: параметр robots пуст. Нужен список наземных роботов '
                             "(например robots:=['ground_1','ground_2']); launch передаёт его всегда.")
        self.model = None
        self.device = 'cpu'
        if g('enabled'):
            self._load(g('model'), g('device'))
        self.latest = {}          # робот -> последний необработанный кадр
        self.rr = 0
        self.n_frames = self.n_boxes = 0
        self.ms_sum = 0.0
        self.pub = self.create_publisher(RgbDetection, '/perception/rgb_detections', 50)
        for r in self.robots:
            self.create_subscription(Image, f'/{r}/rgb/image_raw',
                                     lambda m, r=r: self.latest.__setitem__(r, m), qos_profile_sensor_data)
        self.create_timer(1.0 / float(g('rate_hz')), self._step)
        self.create_timer(60.0, self._stats)

    def _load(self, path, device):
        try:
            from ultralytics import settings
            settings.update({'sync': False})
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'ultralytics settings: {e}')
        import torch
        from ultralytics import YOLO
        if device.startswith('cuda') and not torch.cuda.is_available():
            self.get_logger().warn('CUDA недоступна, YOLO на CPU')
            device = 'cpu'
        self.device = device
        self.model = YOLO(path)
        self.model.predict(np.zeros((240, 320, 3), np.uint8), imgsz=self.imgsz, device=device, verbose=False)  # прогрев
        self.get_logger().info(f'YOLO: {path}, device={device}, imgsz={self.imgsz}, conf={self.conf}')

    def _step(self):
        if self.model is None or not self.robots:
            return
        for _ in range(len(self.robots)):    # следующий робот по кругу, у которого есть новый кадр
            r = self.robots[self.rr % len(self.robots)]
            self.rr += 1
            msg = self.latest.pop(r, None)
            if msg is not None:
                self._infer(r, msg)
                return

    def _infer(self, robot, m):
        if m.encoding != 'rgb8':
            self.get_logger().warn(f'кадр {m.encoding}, нужен rgb8', throttle_duration_sec=30.0)
            return
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        t0 = time.perf_counter()
        # ultralytics для numpy ждёт BGR, а у нас RGB: переворачиваем каналы
        res = self.model.predict(img[:, :, ::-1], imgsz=self.imgsz, conf=self.conf, classes=[0],
                                 device=self.device, verbose=False)[0]
        self.ms_sum += (time.perf_counter() - t0) * 1e3
        self.n_frames += 1
        if res.boxes is None:
            return
        for xyxy, sc in zip(res.boxes.xyxy.cpu().numpy(), res.boxes.conf.cpu().numpy()):
            x0, y0, x1, y1 = (float(v) for v in xyxy)
            b, hw = box_angles(x0, x1, m.width, self.hfov)
            d = RgbDetection()
            d.header = m.header
            d.robot = robot
            d.bearing_rad, d.half_width_rad, d.score = float(b), float(hw), float(sc)
            d.box_px = [int(round(x0)), int(round(y0)), int(round(x1)), int(round(y1))]
            self.pub.publish(d)
            self.n_boxes += 1

    def _stats(self):
        if self.model is None:
            return
        ms = self.ms_sum / self.n_frames if self.n_frames else 0.0
        self.get_logger().info(f'кадров {self.n_frames}, рамок {self.n_boxes}, среднее {ms:.1f} мс')


def main():
    rclpy.init()
    node = RgbDetector()
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

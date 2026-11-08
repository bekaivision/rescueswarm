#!/usr/bin/env python3
"""Нагрузка YOLO для замера VRAM: работает внутри контейнера симуляции (docker exec).
Берёт RGB трёх роботов (/ground_N/rgb/image_raw), крутит инференс с частотой FPS
(по кадру на тик, роботы по кругу) на cuda, печатает итог одной JSON-строкой.
  python3 yolo_vram_load.py --models yolo11n,yolo11s --seconds 45 [--fps 15] [--imgsz 640]
"""
import argparse
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO


class Feed(Node):
    def __init__(self, n):
        super().__init__('yolo_vram_load')
        self.last = {}
        for i in range(1, n + 1):
            self.create_subscription(Image, f'/ground_{i}/rgb/image_raw',
                                     lambda m, i=i: self.last.__setitem__(i, m), 1)


def to_bgr(m):
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
    return a[:, :, ::-1].copy() if m.encoding == 'rgb8' else a.copy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--models', default='yolo11n')
    ap.add_argument('--seconds', type=float, default=45)
    ap.add_argument('--fps', type=float, default=15)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--robots', type=int, default=3)
    a = ap.parse_args()
    rclpy.init()
    node = Feed(a.robots)
    nets = [YOLO(f'/opt/yolo/{m}.pt') for m in a.models.split(',')]
    dummy = np.zeros((240, 320, 3), np.uint8)
    for net in nets:    # прогрев и выделение памяти
        net.predict(dummy, imgsz=a.imgsz, conf=0.25, classes=[0], verbose=False, device=0)
    t_end = time.time() + a.seconds
    period = 1.0 / a.fps
    nxt = time.time()
    lat, k, dets, src = [], 0, 0, 0
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.0)
        if time.time() < nxt:
            time.sleep(0.001)
            continue
        nxt += period
        i = 1 + k % a.robots
        m = node.last.get(i)
        frame = to_bgr(m) if m is not None else dummy
        src += 1 if m is not None else 0
        t0 = time.perf_counter()
        for net in nets:
            r = net.predict(frame, imgsz=a.imgsz, conf=0.25, classes=[0], verbose=False, device=0)[0]
            dets += len(r.boxes)
        lat.append((time.perf_counter() - t0) * 1000)
        k += 1
    print(json.dumps(dict(models=a.models, imgsz=a.imgsz, frames=k, real_frames=src,
                          achieved_fps=k / a.seconds, lat_ms_mean=float(np.mean(lat)),
                          lat_ms_p95=float(np.percentile(lat, 95)), person_boxes=dets)), flush=True)


if __name__ == '__main__':
    main()

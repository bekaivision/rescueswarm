#!/usr/bin/env python3
"""Пишет RTF мира в CSV (wall_time, sim_time, rtf): подписка gz-transport на /world/rescue/stats.
Запускается внутри контейнера симуляции: python3 yolo_vram_rtf.py out.csv seconds"""
import sys
import time

from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node

out, secs = sys.argv[1], float(sys.argv[2])
f = open(out, 'w', buffering=1)
f.write('t,sim_time,rtf\n')
last = [0.0]


def cb(m):
    now = time.time()
    if now - last[0] >= 1.0:
        last[0] = now
        f.write(f'{int(now)},{m.sim_time.sec + m.sim_time.nsec * 1e-9:.3f},{m.real_time_factor:.4f}\n')


node = Node()
node.subscribe(WorldStatistics, '/world/rescue/stats', cb)
time.sleep(secs)

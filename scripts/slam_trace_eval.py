#!/usr/bin/env python3
"""Сравнение slam_trace.csv (поза SLAM и по одним колёсам на штамп скана) с истинной позой из pose_log.csv.
Запуск: python3 scripts/slam_trace_eval.py <run_dir> [шаг_с]. Печатает ошибку SLAM и колёс по времени."""
import bisect
import collections
import csv
import math
import sys

D = sys.argv[1].rstrip('/') + '/'
step = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
gt = collections.defaultdict(list)
for r in csv.DictReader(open(D + 'pose_log.csv')):
    if r['source'] == 'gt':
        gt[r['robot']].append((float(r['t']), float(r['x']), float(r['y'])))
tr = collections.defaultdict(list)
for r in csv.DictReader(open(D + 'slam_trace.csv')):
    tr[r['robot']].append(r)
for rb in sorted(tr):
    ts = [g[0] for g in gt[rb]]
    nxt, out = 0.0, []
    for r in tr[rb]:
        t = float(r['t_sim'])
        if t < nxt:
            continue
        nxt = t + step
        g = gt[rb][min(bisect.bisect(ts, t), len(ts) - 1)]
        es = math.dist(g[1:], (float(r['slam_x']), float(r['slam_y'])))
        ew = math.dist(g[1:], (float(r['wheel_x']), float(r['wheel_y'])))
        out.append(f't={t:.0f} slam={es:.2f} wheel={ew:.2f}')
    print(rb, ' | '.join(out))

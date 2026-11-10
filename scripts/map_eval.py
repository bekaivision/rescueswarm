#!/usr/bin/env python3
"""Качество итоговой карты /map против эталона мира (хост, numpy; scipy по возможности).

  python3 scripts/map_eval.py <run_dir> [--out map_eval.json]

Берёт <run_dir>/final_map.npy (карта на конец прогона, строка 0 = низ, -1/0/100; её origin и
resolution в final_map.yaml) и эталон <run_dir>/world/gt_map.pgm + gt_map.yaml (истинные стены и
завалы, растеризованные генератором rs_worlds в ту же сетку; генератор сам пишет ground_truth.yaml
с описанием коробок, а gt_map.pgm - их растр). Метрики (формулы в rs_eval/metrics.py: map_metrics):
точность и полнота занятых клеток с допуском 1 и 2 клетки, клетки внутри препятствий, ошибочно
свободные, среднее расстояние от занятых клеток карты до ближайшего препятствия.
Результат печатается и пишется в <run_dir>/map_eval.json.
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'rs_eval'))
from rs_eval import metrics  # noqa: E402


def read_yaml(path):
    try:
        import yaml
        with open(path) as f:
            return yaml.safe_load(f)
    except ImportError:    # простой разбор плоского yaml
        out = {}
        with open(path) as f:
            for line in f:
                if ':' in line and not line.startswith('#'):
                    k, v = line.split(':', 1)
                    v = v.strip()
                    if v.startswith('['):
                        v = [float(x) for x in v.strip('[]').split(',')]
                    else:
                        try:
                            v = float(v)
                        except ValueError:
                            pass
                    out[k.strip()] = v
        return out


def read_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        w, h = (int(v) for v in f.readline().split())
        f.readline()
        return np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dir')
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    rd = a.run_dir
    est_p = os.path.join(rd, 'final_map.npy')
    gt_yaml = os.path.join(rd, 'world', 'gt_map.yaml')
    if not os.path.exists(est_p) or not os.path.exists(gt_yaml):
        sys.exit(f'нужны {est_p} и {gt_yaml}')
    est = np.load(est_p)
    gm = read_yaml(gt_yaml)
    gt_occ = (read_pgm(os.path.join(rd, 'world', gm['image'])) < 128)[::-1]   # строка 0 = низ, как в OccupancyGrid
    res = float(gm['resolution'])
    em_p = os.path.join(rd, 'final_map.yaml')
    if os.path.exists(em_p):
        em = read_yaml(em_p)
        if abs(float(em['resolution']) - res) > 1e-9 or any(abs(float(u) - float(v)) > 1e-6 for u, v in
                                                            zip(em['origin'][:2], gm['origin'][:2])):
            sys.exit(f'сетки карты и эталона различаются: {em["origin"]}/{em["resolution"]} vs '
                     f'{gm["origin"]}/{gm["resolution"]}')
    m = metrics.map_metrics(est, gt_occ, res)
    m['resolution'] = res
    m['known_fraction'] = float((est >= 0).mean())
    for k, v in m.items():
        print(f'{k:24} {v:.4f}' if isinstance(v, float) else f'{k:24} {v}')
    out = a.out or os.path.join(rd, 'map_eval.json')
    with open(out, 'w') as f:
        json.dump(m, f, indent=1)
    print(f'записано: {out}')


if __name__ == '__main__':
    main()

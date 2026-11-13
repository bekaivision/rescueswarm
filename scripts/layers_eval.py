#!/usr/bin/env python3
"""Слои карты воздух/земля (Фаза 5, RQ3): насколько совпадают карта дронов и карта наземных роботов.

  python3 scripts/layers_eval.py <run_dir> [--out layers_eval.json]

Берёт <run_dir>/final_map_ground.npy и final_map_air.npy (их пишет rs_eval/coverage в конце прогона со
смешанным роем, та же сетка, что у final_map.npy) и эталон <run_dir>/world/gt_map.pgm.
Считает:
  - качество каждого слоя и общей карты против эталона (те же метрики, что scripts/map_eval.py);
  - согласие слоёв: занятые клетки одного слоя, у которых в радиусе 2 клеток есть занятая клетка
    другого, только там, где оба слоя что-то видели;
  - сдвиг воздушного слоя относительно наземного: перебор сдвигов до +-max_shift м с шагом в клетку,
    выбирается сдвиг с наибольшим числом совпавших занятых клеток. С истинной позой ожидаем (0, 0);
    с оценкой позы дрона (PX4 EKF) сдвиг показывает систематическую ошибку совмещения.
Результат печатается и пишется в <run_dir>/layers_eval.json. Только numpy и scipy (если есть).
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'rs_eval'))
from map_eval import read_pgm, read_yaml  # noqa: E402
from rs_eval import metrics  # noqa: E402


def dilate(mask, k):
    """Квадратная дилатация на k клеток (без scipy)."""
    out = mask.copy()
    for dy in range(-k, k + 1):
        for dx in range(-k, k + 1):
            out |= np.roll(np.roll(mask, dy, axis=0), dx, axis=1)
    return out


def shift(mask, dy, dx):
    """Сдвиг без заворачивания: пустые края."""
    out = np.zeros_like(mask)
    h, w = mask.shape
    ys, yd = (slice(0, h - dy), slice(dy, h)) if dy >= 0 else (slice(-dy, h), slice(0, h + dy))
    xs, xd = (slice(0, w - dx), slice(dx, w)) if dx >= 0 else (slice(-dx, w), slice(0, w + dx))
    out[yd, xd] = mask[ys, xs]
    return out


def best_shift(air_occ, ground_occ, both_known, max_cells, tol=1):
    """Сдвиг (dy, dx) воздушного слоя, при котором больше всего его занятых клеток ложится на занятые
    клетки наземного (с допуском tol) в общей видимой области. Возвращает сдвиг и долю совпадения."""
    g = dilate(ground_occ, tol)
    best = (0, 0, -1.0)
    base = None
    for dy in range(-max_cells, max_cells + 1):
        for dx in range(-max_cells, max_cells + 1):
            a = shift(air_occ, dy, dx) & both_known
            n = int(a.sum())
            if n == 0:
                continue
            frac = float((a & g).sum()) / n
            if dy == 0 and dx == 0:
                base = frac
            if frac > best[2] + 1e-9 or (abs(frac - best[2]) <= 1e-9 and dy * dy + dx * dx < best[0] ** 2 + best[1] ** 2):
                best = (dy, dx, frac)
    return best, base


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dir')
    ap.add_argument('--out', default='')
    ap.add_argument('--max-shift', type=float, default=1.0, help='м')
    a = ap.parse_args()
    rd = a.run_dir
    paths = {k: os.path.join(rd, f'final_map_{k}.npy') for k in ('ground', 'air')}
    if not all(os.path.exists(p) for p in paths.values()):
        sys.exit(f'нет слоёв карты в {rd} (прогон без дронов?)')
    gm = read_yaml(os.path.join(rd, 'world', 'gt_map.yaml'))
    gt_occ = (read_pgm(os.path.join(rd, 'world', gm['image'])) < 128)[::-1]
    res = float(gm['resolution'])
    layers = {k: np.load(p) for k, p in paths.items()}
    if os.path.exists(os.path.join(rd, 'final_map.npy')):
        layers['fused'] = np.load(os.path.join(rd, 'final_map.npy'))
    out = {'resolution': res, 'layers': {}}
    for k, d in layers.items():
        m = metrics.map_metrics(d, gt_occ, res)
        m['known_cells'] = int((d >= 0).sum())
        out['layers'][k] = m
    g, ai = layers['ground'], layers['air']
    both = (g >= 0) & (ai >= 0)
    g_occ, a_occ = g >= 50, ai >= 50
    gd, ad = dilate(g_occ, 2), dilate(a_occ, 2)
    sel_a, sel_g = a_occ & both, g_occ & both
    out['both_known_cells'] = int(both.sum())
    out['agree_air_on_ground_tol2'] = float((sel_a & gd).sum() / max(sel_a.sum(), 1))
    out['agree_ground_on_air_tol2'] = float((sel_g & ad).sum() / max(sel_g.sum(), 1))
    # где слои спорят: наземный слой свободен, воздушный занят (и наоборот), вне допуска 2 клетки
    out['air_occ_ground_free_cells'] = int((a_occ & (g == 0) & ~gd).sum())
    out['ground_occ_air_free_cells'] = int((g_occ & (ai == 0) & ~ad).sum())
    mc = int(round(a.max_shift / res))
    (dy, dx, frac), base = best_shift(a_occ, g_occ, both, mc)
    out['best_shift_m'] = [dx * res, dy * res]    # (x, y): строки - y, столбцы - x
    out['best_shift_match'] = frac
    out['zero_shift_match'] = base
    for k, v in out.items():
        if k != 'layers':
            print(f'{k:28} {v}')
    for k, m in out['layers'].items():
        print(f'{k:8} P@1 {m["precision_tol1"]:.3f} P@2 {m["precision_tol2"]:.3f} '
              f'R@2 {m["recall_tol2"]:.3f} wrong_free {m["wrong_free_cells"]} known {m["known_cells"]}')
    p = a.out or os.path.join(rd, 'layers_eval.json')
    with open(p, 'w') as f:
        json.dump(out, f, indent=1)
    print(f'записано: {p}')


if __name__ == '__main__':
    main()

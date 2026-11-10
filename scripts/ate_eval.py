#!/usr/bin/env python3
"""ATE и RPE локализации по журналу поз прогона (хост, нужен только numpy).

  python3 scripts/ate_eval.py <run_dir> [--align] [--out ate.json]

Читает <run_dir>/pose_log.csv (t,robot,source,x,y,yaw; source = pose | gt, пишет узел coverage):
pose - мировая поза, которой пользовалась система, gt - истинная поза (/gt_odom). Истинная поза
интерполируется во времена pose. Обе позы в кадре world, поэтому по умолчанию выравнивания нет;
--align добавляет выравнивание SE(2) по Умеяме (без масштаба) на каждого робота.
Для каждого робота: ATE (RMSE, среднее, максимум), ошибка в конце, RPE на 1 м и 5 м истинного пути.
Результат печатается таблицей и пишется в <run_dir>/ate.json.
"""
import argparse
import csv
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'rs_eval'))
from rs_eval import metrics  # noqa: E402


def load_log(path):
    data = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            data.setdefault(row['robot'], {}).setdefault(row['source'], []).append(
                (float(row['t']), float(row['x']), float(row['y']), float(row['yaw'])))
    return {r: {s: np.array(v) for s, v in d.items()} for r, d in data.items()}


def evaluate(log, align=False, deltas=(1.0, 5.0)):
    res = {}
    for r in sorted(log):
        d = log[r]
        if 'pose' not in d or 'gt' not in d:
            continue
        p, g = d['pose'], d['gt']
        p = p[np.argsort(p[:, 0])]
        g = g[np.argsort(g[:, 0])]
        gi = metrics.interp_poses(g[:, 0], g[:, 1:], p[:, 0])
        ok = ~np.isnan(gi[:, 0])
        est, gt, tt = p[ok, 1:], gi[ok], p[ok, 0]
        if len(est) < 2:
            continue
        info = {'duration_s': float(tt[-1] - tt[0]), 'path_gt_m': float(metrics.path_length(gt)[-1])}
        if align:
            R, t = metrics.align_se2(est[:, :2], gt[:, :2])
            est = metrics.apply_se2(R, t, est)
            info['align_yaw_deg'] = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
        a, err = metrics.ate(est, gt)
        info['ate'] = a
        info['rpe'] = {f'{dl:g}m': metrics.rpe(est, gt, dl) for dl in deltas}
        # рост ошибки: средняя ошибка по четвертям времени
        q = np.array_split(err, 4)
        info['ate_by_quarter_mean'] = [float(x.mean()) if len(x) else float('nan') for x in q]
        res[r] = info
    return res


def print_table(res, align):
    print(f'ATE по позициям, м{" (выравнивание SE2)" if align else ""}')
    print(f'{"robot":9} {"n":>6} {"path":>7} {"rmse":>8} {"mean":>8} {"max":>8} {"final":>8}'
          f' {"RPE1m":>8} {"RPE5m":>8} {"rot5m,deg":>9}')
    for r, i in res.items():
        a = i['ate']
        r1, r5 = i['rpe'].get('1m'), i['rpe'].get('5m')
        f = lambda v, k: f'{v[k]:8.3f}' if v else f'{"-":>8}'  # noqa: E731
        print(f'{r:9} {a["n"]:6d} {i["path_gt_m"]:7.1f} {a["rmse"]:8.3f} {a["mean"]:8.3f} {a["max"]:8.3f} '
              f'{a["final"]:8.3f} {f(r1, "trans_rmse")} {f(r5, "trans_rmse")} '
              f'{(r5["rot_rmse_deg"] if r5 else float("nan")):9.2f}')
    if len(res) > 1:
        rm = np.sqrt(np.mean([i['ate']['rmse'] ** 2 for i in res.values()]))
        print(f'{"все":9} {"":6} {"":7} {rm:8.3f}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dir')
    ap.add_argument('--align', action='store_true', help='выравнивание SE2 (Умеяма) на каждого робота')
    ap.add_argument('--out', default='', help='куда писать json (по умолчанию <run_dir>/ate.json)')
    a = ap.parse_args()
    path = os.path.join(a.run_dir, 'pose_log.csv')
    if not os.path.exists(path):
        sys.exit(f'нет {path}: прогон сделан без журнала поз (нужен узел coverage новой версии)')
    res = evaluate(load_log(path), a.align)
    if not res:
        sys.exit('в pose_log.csv нет пар pose+gt')
    print_table(res, a.align)
    out = a.out or os.path.join(a.run_dir, 'ate.json')
    with open(out, 'w') as f:
        json.dump({'align': a.align, 'robots': res,
                   'rmse_all': float(np.sqrt(np.mean([i['ate']['rmse'] ** 2 for i in res.values()])))},
                  f, indent=1)
    print(f'записано: {out}')


if __name__ == '__main__':
    main()

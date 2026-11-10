#!/usr/bin/env python3
"""Сводка проверки Фазы 4: сравнение gt и slam по seed (хост, stdlib).

  python3 scripts/phase4_summary.py <check_dir> "<seeds>"

Читает в <check_dir>/seed_<S>_{gt,slam}/ файлы check.json (phase3_eval), ate.json (ate_eval),
map_eval.json (map_eval), coverage.csv, rtf.txt, load.txt. Пишет <check_dir>/summary.md и summary.json,
печатает строки OK/FAIL по порогам. Код возврата 1, если есть FAIL.

Пороги (предложены заранее, не подгонялись под результат):
  1. завершение: каждый slam-прогон завершён до max_time, покрытие >= 90 %;
  2. люди: сумма CONFIRMED достижимых людей в slam >= 90 % от суммы в gt (по всем seed);
  3. ловушки: CONFIRMED ловушек в slam = 0 (по всем seed);
  4. ATE: среднее по seed от RMSE (по всем роботам прогона) в slam < 0.5 м;
  5. карта: средняя по seed точность занятых клеток с допуском 2 клетки (0.10 м) в slam >= 0.7.
Справочно, без порога: ghost CONFIRMED, ошибка позиции людей, RPE, RTF, нагрузка.
"""
import csv
import json
import math
import os
import re
import statistics as st
import sys

MODES = ('gt', 'slam')


def jl(p):
    try:
        return json.load(open(p))
    except Exception:  # noqa: BLE001
        return None


def t95(p):
    try:
        for r in csv.DictReader(open(p)):
            if float(r['coverage']) >= 0.95:
                return float(r['sim_time_s'])
    except Exception:  # noqa: BLE001
        pass
    return None


def floats(p, rx=None):
    out = []
    try:
        for l in open(p):
            if rx:
                m = re.search(rx, l)
                if m:
                    out.append(float(m[1]))
            elif l.strip():
                out.append(float(l.split()[0]))
    except Exception:  # noqa: BLE001
        pass
    return out


def collect(chk, s, m):
    d = f'{chk}/seed_{s}_{m}'
    c, a, mp = jl(f'{d}/check.json'), jl(f'{d}/ate.json'), jl(f'{d}/map_eval.json')
    r = {'seed': s, 'mode': m, 'have': c is not None}
    if c is None:
        return r
    rtf = floats(f'{d}/rtf.txt', r'real_time_factor:\s*([\d.eE+-]+)')
    load = floats(f'{d}/load.txt')
    r.update(
        coverage=c['final_coverage'], t95=t95(f'{d}/coverage.csv'), completion_t=c['completion_t'],
        completion_reason=c['completion_reason'],
        people=c['people'], reachable=c['reachable'], confirmed=c['confirmed'],
        confirmed_reachable=c['confirmed_reachable'], decoys=c['decoys'],
        decoys_confirmed=len(c['decoys_confirmed']), ghosts_confirmed=c['ghosts']['CONFIRMED'],
        ghosts_total=sum(c['ghosts'].values()),
        loc_err_mean=c['loc_err_mean'], loc_err_max=c['loc_err_max'],
        rtf_mean=round(st.mean(rtf), 3) if rtf else None, rtf_min=round(min(rtf), 3) if rtf else None,
        load_mean=round(st.mean(load), 1) if load else None, load_max=round(max(load), 1) if load else None,
        phase3_fails=c['fails'])
    if a:
        rb = a['robots']
        r['ate'] = {k: [round(v['ate']['rmse'], 3), round(v['ate']['max'], 3)] for k, v in rb.items()}
        r['ate_rmse'] = round(a['rmse_all'], 3)
        r['ate_max'] = round(max(v['ate']['max'] for v in rb.values()), 3)
        r['rpe1'] = round(st.mean(v['rpe']['1m']['trans_rmse'] for v in rb.values() if v['rpe'].get('1m')), 3)
    if mp:
        for k in ('precision_tol1', 'recall_tol1', 'precision_tol2', 'recall_tol2'):
            r[k] = round(mp[k], 3)
    return r


def f(v, n=2):
    return '-' if v is None else (f'{v:.{n}f}' if isinstance(v, float) else str(v))


def mean_of(rows, k):
    v = [r[k] for r in rows if r.get(k) is not None]
    return (st.mean(v), min(v), max(v)) if v else (None, None, None)


def main():
    chk, seeds = sys.argv[1], sys.argv[2].split()
    runs = {(s, m): collect(chk, s, m) for s in seeds for m in MODES}
    cols = [('coverage', 'покрытие, %', lambda r: f(r['coverage'] * 100, 1)),
            ('t95', 't95, с', lambda r: f(r['t95'], 0)),
            ('ate', 'ATE rmse / max по роботам, м', lambda r: ' ; '.join(f'{v[0]:.2f}/{v[1]:.2f}' for v in r['ate'].values()) if 'ate' in r else '-'),
            ('rpe1', 'RPE 1 м, м', lambda r: f(r.get('rpe1'), 3)),
            ('p1', 'карта P@1 / R@1', lambda r: f"{f(r.get('precision_tol1'))} / {f(r.get('recall_tol1'))}"),
            ('p2', 'карта P@2 / R@2', lambda r: f"{f(r.get('precision_tol2'))} / {f(r.get('recall_tol2'))}"),
            ('ppl', 'люди CONFIRMED (достижимых) / достижимых', lambda r: f"{r['confirmed']} ({r['confirmed_reachable']}) / {r['reachable']}"),
            ('dec', 'ловушки CONFIRMED', lambda r: str(r['decoys_confirmed'])),
            ('gh', 'ghost CONFIRMED', lambda r: str(r['ghosts_confirmed'])),
            ('loc', 'ошибка позиции людей ср./макс, м', lambda r: f"{f(r['loc_err_mean'])} / {f(r['loc_err_max'])}"),
            ('rtf', 'RTF ср./мин', lambda r: f"{f(r['rtf_mean'])} / {f(r['rtf_min'])}"),
            ('load', 'load1 ср./макс', lambda r: f"{f(r['load_mean'], 1)} / {f(r['load_max'], 1)}")]
    L = [f'# Проверка Фазы 4 (seeds {" ".join(seeds)}): gt против slam\n',
         '| seed | режим | ' + ' | '.join(c[1] for c in cols) + ' |', '| --- | --- | ' + ' | '.join('---' for _ in cols) + ' |']
    for s in seeds:
        for m in MODES:
            r = runs[(s, m)]
            if not r['have']:
                L.append(f'| {s} | {m} | нет данных |' + ' |' * (len(cols) - 1))
            else:
                L.append(f'| {s} | {m} | ' + ' | '.join(c[2](r) for c in cols) + ' |')
    L.append('\n## Среднее (мин / макс) по seed\n')
    L.append('| режим | покрытие, % | t95, с | ATE rmse, м | ATE max, м | RPE 1 м | P@1 | R@1 | P@2 | R@2 | люди достижимых CONFIRMED | ловушки CONFIRMED | ghost CONFIRMED | RTF | load1 |')
    L.append('| --- ' * 15 + '|')
    agg = {}
    for m in MODES:
        rs = [runs[(s, m)] for s in seeds if runs[(s, m)]['have']]
        a = {k: mean_of(rs, k) for k in ('coverage', 't95', 'ate_rmse', 'ate_max', 'rpe1', 'precision_tol1', 'recall_tol1',
                                         'precision_tol2', 'recall_tol2', 'rtf_mean', 'load_mean')}
        a['reach_conf'] = sum(r['confirmed_reachable'] for r in rs)
        a['reach'] = sum(r['reachable'] for r in rs)
        a['decoys_conf'] = sum(r['decoys_confirmed'] for r in rs)
        a['ghosts_conf'] = sum(r['ghosts_confirmed'] for r in rs)
        a['n'] = len(rs)
        agg[m] = a

        def cell(k, n=2, scale=1.0):
            mn, lo, hi = a[k]
            return '-' if mn is None else f'{mn * scale:.{n}f} ({lo * scale:.{n}f} / {hi * scale:.{n}f})'
        L.append(f"| {m} | {cell('coverage', 1, 100)} | {cell('t95', 0)} | {cell('ate_rmse')} | {cell('ate_max')} | {cell('rpe1', 3)} | "
                 f"{cell('precision_tol1')} | {cell('recall_tol1')} | {cell('precision_tol2')} | {cell('recall_tol2')} | "
                 f"{a['reach_conf']}/{a['reach']} | {a['decoys_conf']} | {a['ghosts_conf']} | {cell('rtf_mean')} | {cell('load_mean', 1)} |")
    # пороги
    sl, gt = agg['slam'], agg['gt']
    checks = []
    bad = [f"seed {s}" for s in seeds if not runs[(s, 'slam')]['have']
           or runs[(s, 'slam')]['completion_t'] is None or 'max_time' in (runs[(s, 'slam')]['completion_reason'] or '')
           or runs[(s, 'slam')]['coverage'] < 0.90]
    checks.append(('slam-прогоны завершены до max_time, покрытие >= 90 %', not bad, 'проблемные: ' + (', '.join(bad) or 'нет')))
    ratio = sl['reach_conf'] / gt['reach_conf'] if gt['reach_conf'] else None
    checks.append(('люди: slam CONFIRMED достижимых >= 90 % от gt', ratio is not None and ratio >= 0.9,
                   f"slam {sl['reach_conf']} / gt {gt['reach_conf']} = {f(ratio * 100 if ratio is not None else None, 1)} %"))
    checks.append(('ловушки: slam CONFIRMED = 0', sl['decoys_conf'] == 0, f"{sl['decoys_conf']}"))
    ate = sl['ate_rmse'][0]
    checks.append(('ATE rmse (среднее по seed) в slam < 0.5 м', ate is not None and ate < 0.5, f(ate, 3)))
    p2 = sl['precision_tol2'][0]
    checks.append(('карта: P@2 (среднее по seed) в slam >= 0.7', p2 is not None and p2 >= 0.7, f(p2, 3)))
    L.append('\n## Пороги\n')
    fails = []
    for name, ok, val in checks:
        line = f"{'OK' if ok else 'FAIL'}: {name} ({val})"
        L.append(f'- {line}')
        print(line)
        if not ok:
            fails.append(name)
    L.append('\nСправочно: gt-прогоны не используют SLAM, их ATE около нуля по построению; '
             'IMU с шумом (imu_noise=true), колёсная одометрия идеальна кроме проскальзывания.')
    open(f'{chk}/summary.md', 'w').write('\n'.join(L) + '\n')
    json.dump({'seeds': seeds, 'runs': {f'{s}_{m}': r for (s, m), r in runs.items()},
               'aggregate': agg, 'checks': [{'name': n, 'ok': o, 'value': v} for n, o, v in checks]},
              open(f'{chk}/summary.json', 'w'), indent=1, ensure_ascii=False)
    print('\n'.join(L))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())

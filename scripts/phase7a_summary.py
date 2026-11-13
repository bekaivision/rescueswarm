#!/usr/bin/env python3
"""Сводка проверки Фазы 7a: смешанный рой + SLAM + связь medium (хост, stdlib + numpy).

  python3 scripts/phase7a_summary.py <check_dir> "<seeds>"

Читает в <check_dir>/seed_<S>_{light,px4}/, n5_seed_1/, n5_seed_1_ground/ файлы check.json (phase3_eval),
map_eval.json, ate.json, layers_eval.json, comms_eval.json, coverage.csv, pose_log.csv, rtf.txt, load.txt,
retries.txt, run_meta.json. Пишет <check_dir>/summary.md и summary.json, печатает строки OK/FAIL/SKIP.
Код возврата 1, если есть FAIL (SKIP - часть не запускалась - не ошибка).

Пороги (заданы до прогонов), отдельно для частей light и px4:
  (a) сумма CONFIRMED достижимых людей по seed >= достижимых - 1;
  (b) ловушек CONFIRMED в каждом прогоне 0;
  (c) ghost CONFIRMED <= 1 в каждом прогоне;
  (d) каждый прогон набрал покрытие >= 90 % до max_time.
Часть n5 (N=5): (e) прогон завершён (EXPLORATION COMPLETE или покрытие >= 90 %) и каждый робот проехал >= 5 м.
"""
import csv
import json
import math
import os
import re
import statistics as st
import sys

MIN_MOVE_M = 5.0


def jl(p):
    try:
        return json.load(open(p))
    except Exception:  # noqa: BLE001
        return None


def t_cov(p, col, thr):
    try:
        for r in csv.DictReader(open(p)):
            if r.get(col) not in (None, '') and float(r[col]) >= thr:
                return float(r['sim_time_s'])
    except Exception:  # noqa: BLE001
        pass
    return None


def floats(p, rx=None):
    out = []
    try:
        for line in open(p):
            if rx:
                m = re.search(rx, line)
                if m:
                    out.append(float(m[1]))
            elif line.strip():
                out.append(float(line.split()[0]))
    except Exception:  # noqa: BLE001
        pass
    return out


def robot_paths(d):
    """Путь каждого робота по истинной позе из pose_log.csv (source=gt), м."""
    last, path = {}, {}
    try:
        for r in csv.DictReader(open(f'{d}/pose_log.csv')):
            if r.get('source') != 'gt':
                continue
            k, x, y = r['robot'], float(r['x']), float(r['y'])
            if k in last:
                path[k] = path.get(k, 0.0) + math.hypot(x - last[k][0], y - last[k][1])
            else:
                path[k] = 0.0
            last[k] = (x, y)
    except Exception:  # noqa: BLE001
        return None
    return {k: round(v, 1) for k, v in path.items()}


def collect(d, label):
    c = jl(f'{d}/check.json')
    r = {'label': label, 'dir': d, 'have': c is not None}
    if c is None:
        return r
    rtf = floats(f'{d}/rtf.txt', r'real_time_factor:\s*([\d.eE+-]+)')
    load = floats(f'{d}/load.txt')
    unreach = [p for p in c['per_person'] if not p['reachable']]
    cov = f'{d}/coverage.csv'
    log = f'{d}/launch.log'
    r.update(coverage=c['final_coverage'], t90=t_cov(cov, 'coverage', 0.90), end_t=c['completion_t'],
             completion_reason=c['completion_reason'], reachable=c['reachable'],
             confirmed_reachable=c['confirmed_reachable'], confirmed=c['confirmed'],
             unreachable=len(unreach),
             unreachable_marked=sum(1 for p in unreach if p['status'] in ('CANDIDATE', 'CONFIRMED')),
             traps_confirmed=len(c['decoys_confirmed']), ghosts_confirmed=c['ghosts']['CONFIRMED'],
             rtf_mean=round(st.mean(rtf), 3) if rtf else None, rtf_min=round(min(rtf), 3) if rtf else None,
             load_mean=round(st.mean(load), 1) if load else None)
    r['finished'] = os.path.exists(log) and 'EXPLORATION COMPLETE' in open(log, errors='ignore').read()
    try:
        r['retries'] = int(open(f'{d}/retries.txt').read().strip())
    except Exception:  # noqa: BLE001
        r['retries'] = None
    meta = jl(f'{d}/run_meta.json') or {}
    r['max_sim_s'] = meta.get('MAX_SIM_S')
    r['n'] = meta.get('N')
    r['wall_s'] = meta.get('wall_s')
    mp = jl(f'{d}/map_eval.json')
    if mp:
        r['p2'] = round(mp['precision_tol2'], 3)
    a = jl(f'{d}/ate.json')
    if a:
        r['ate_rmse'] = round(a['rmse_all'], 3)
        r['ate_max'] = round(max(v['ate']['max'] for v in a['robots'].values()), 3)
    le = jl(f'{d}/layers_eval.json')
    if le:
        sh = le['best_shift_m']
        r['shift_m'] = round(math.hypot(sh[0], sh[1]), 3)
    ce = jl(f'{d}/comms_eval.json')
    if ce:
        r['path_base'] = (ce.get('connectivity') or {}).get('path_to_base_mean')
    r['paths'] = robot_paths(d)
    return r


def f(v, n=2):
    return '-' if v is None else (f'{v:.{n}f}' if isinstance(v, float) else str(v))


def pct(v, n=1):
    return '-' if v is None else f'{v * 100:.{n}f}'


COLS = [('покрытие, %', lambda r: pct(r['coverage'])),
        ('t90, с', lambda r: f(r['t90'], 0)),
        ('конец, с', lambda r: f(r['end_t'], 0)),
        ('люди достижимых / всего', lambda r: f"{r['confirmed_reachable']} / {r['reachable']}"),
        ('недостижимые отмечены', lambda r: f"{r['unreachable_marked']} / {r['unreachable']}"),
        ('ловушки CONFIRMED', lambda r: str(r['traps_confirmed'])),
        ('ghost CONFIRMED', lambda r: str(r['ghosts_confirmed'])),
        ('P@2 карты', lambda r: f(r.get('p2'))),
        ('ATE rmse / max, м', lambda r: f"{f(r.get('ate_rmse'))} / {f(r.get('ate_max'))}" if 'ate_rmse' in r else '-'),
        ('сдвиг слоёв, м', lambda r: f(r.get('shift_m'))),
        ('путь до базы, %', lambda r: pct(r.get('path_base'), 0)),
        ('RTF ср. / мин', lambda r: f"{f(r['rtf_mean'])} / {f(r['rtf_min'])}"),
        ('load1', lambda r: f(r['load_mean'], 1)),
        ('повторы', lambda r: f(r['retries']))]


def table(rows):
    L = ['| прогон | ' + ' | '.join(c[0] for c in COLS) + ' |', '| --- | ' + ' | '.join('---' for _ in COLS) + ' |']
    for r in rows:
        if not r['have']:
            L.append(f"| {r['label']} | нет данных |" + ' |' * (len(COLS) - 1))
        else:
            L.append(f"| {r['label']} | " + ' | '.join(c[1](r) for c in COLS) + ' |')
    return L


def main():
    chk, seeds = sys.argv[1].rstrip('/'), sys.argv[2].split()
    parts = {p: [collect(f'{chk}/seed_{s}_{p}', f'seed {s} {p}') for s in seeds] for p in ('light', 'px4')}
    n5 = [collect(f'{chk}/n5_seed_1', 'n5 смешанный'), collect(f'{chk}/n5_seed_1_ground', 'n5 наземные')]
    ran = {p: any(os.path.isdir(f"{chk}/seed_{s}_{p}") for s in seeds) for p in parts}
    ran['n5'] = any(os.path.isdir(f'{chk}/{x}') for x in ('n5_seed_1', 'n5_seed_1_ground'))

    L = [f'# Проверка Фазы 7a (seeds {" ".join(seeds)}): SLAM + дроны + связь medium\n']
    for p, rows in parts.items():
        L.append(f'## Часть {p}\n')
        L += table(rows) if ran[p] else ['Не запускалась.']
        L.append('')
    L.append('## Часть n5 (N=5, seed 1)\n')
    if ran['n5']:
        L += table(n5)
        L.append('\n| прогон | путь каждого робота, м |\n| --- | --- |')
        for r in n5:
            L.append(f"| {r['label']} | {', '.join(f'{k}: {v}' for k, v in sorted((r.get('paths') or {}).items())) or '-'} |")
    else:
        L.append('Не запускалась.')

    checks = []   # (имя, статус OK|FAIL|SKIP, значение)
    for p, rows in parts.items():
        if not ran[p]:
            for k in 'abcd':
                checks.append((f'{p}: ({k})', 'SKIP', 'часть не запускалась'))
            continue
        have = [r for r in rows if r['have']]
        miss = [r['label'] for r in rows if not r['have']]
        tot = sum(r['reachable'] for r in have)
        got = sum(r['confirmed_reachable'] for r in have)
        # недостающие прогоны делают проверку (a) невыполнимой: порог считаем по всем seed
        checks.append((f'{p} (a): люди достижимых CONFIRMED >= достижимых - 1', 'OK' if not miss and got >= tot - 1 else 'FAIL',
                       f'{got} / {tot}, порог {tot - 1}' + (f'; нет данных: {", ".join(miss)}' if miss else '')))
        bad = [f"{r['label']}: {r['traps_confirmed']}" for r in have if r['traps_confirmed'] > 0]
        checks.append((f'{p} (b): ловушек CONFIRMED 0 в каждом прогоне', 'OK' if not bad and not miss else 'FAIL',
                       ', '.join(bad) or ('нет данных: ' + ', '.join(miss) if miss else 'везде 0')))
        bad = [f"{r['label']}: {r['ghosts_confirmed']}" for r in have if r['ghosts_confirmed'] > 1]
        checks.append((f'{p} (c): ghost CONFIRMED <= 1 в каждом прогоне', 'OK' if not bad and not miss else 'FAIL',
                       ', '.join(bad) or ('нет данных: ' + ', '.join(miss) if miss else 'везде <= 1')))
        bad = [r['label'] for r in have if r['t90'] is None or r['coverage'] < 0.90
               or (r.get('max_sim_s') and r['t90'] >= r['max_sim_s'])]
        checks.append((f'{p} (d): покрытие >= 90 % до max_time в каждом прогоне', 'OK' if not bad and not miss else 'FAIL',
                       'не дошли: ' + (', '.join(bad + miss) or 'нет')))
    if not ran['n5']:
        checks.append(('n5 (e)', 'SKIP', 'часть не запускалась'))
    else:
        bad = []
        for r in n5:
            if not os.path.isdir(r['dir']):
                continue
            if not r['have']:
                bad.append(f"{r['label']}: нет данных")
                continue
            if not (r['finished'] or r['coverage'] >= 0.90):
                bad.append(f"{r['label']}: не завершён (покрытие {pct(r['coverage'])} %)")
            paths = r.get('paths') or {}
            exp = r.get('n') or 5
            slow = [k for k, v in paths.items() if v < MIN_MOVE_M]
            if len(paths) < exp:
                bad.append(f"{r['label']}: в pose_log {len(paths)} из {exp} роботов")
            if slow:
                bad.append(f"{r['label']}: проехали < {MIN_MOVE_M:.0f} м: {', '.join(slow)}")
        checks.append((f'n5 (e): прогон завершён, каждый робот проехал >= {MIN_MOVE_M:.0f} м', 'OK' if not bad else 'FAIL',
                       '; '.join(bad) or 'да'))

    L.append('\n## Пороги\n')
    fails = 0
    for name, status, val in checks:
        line = f'{status}: {name} ({val})'
        L.append(f'- {line}')
        print(line)
        fails += status == 'FAIL'
    open(f'{chk}/summary.md', 'w').write('\n'.join(L) + '\n')
    slim = lambda rows: [{k: v for k, v in r.items() if k != 'dir'} for r in rows]  # noqa: E731
    json.dump({'seeds': seeds, 'light': slim(parts['light']), 'px4': slim(parts['px4']), 'n5': slim(n5),
               'checks': [{'name': n, 'status': s, 'value': v} for n, s, v in checks]},
              open(f'{chk}/summary.json', 'w'), indent=1, ensure_ascii=False)
    print('\n'.join(L))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())

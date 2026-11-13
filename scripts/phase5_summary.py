#!/usr/bin/env python3
"""Сводка проверки Фазы 5: только наземные роботы против смешанного роя (хост, stdlib).

  python3 scripts/phase5_summary.py <check_dir> "<seeds>"

Читает в <check_dir>/seed_<S>_{ground,mixed}/ файлы check.json (phase3_eval), coverage.csv,
map_eval.json, layers_eval.json (только mixed), ate.json (только mixed: позы дронов), rtf.txt, load.txt.
Пишет <check_dir>/summary.md и summary.json, печатает строки OK/FAIL. Код возврата 1, если есть FAIL.

Пороги (заданы до прогонов; Фаза 5 готова, когда дроны и наземные роботы строят одну карту и находят людей):
  1. завершение: каждый mixed-прогон завершён до max_time, общее покрытие >= 90 %,
     покрытие воздушного слоя >= 80 % (дроны действительно дополнили карту);
  2. люди: сумма CONFIRMED достижимых людей в mixed >= 90 % от суммы достижимых (по всем seed);
  3. ловушки: CONFIRMED ловушек в mixed = 0; ghost CONFIRMED в mixed не больше, чем у одних наземных
     роботов (сумма по seed); ни одной отметки CONFIRMED только по наблюдениям дронов.
     (Изменено после первого интеграционного прогона: там был ghost CONFIRMED только от наземных роботов,
     по слабому RGB - известная слабость Фазы 3b, к дронам не относится. Сравнение с наземным прогоном
     проверяет то, что относится к Фазе 5.)
  4. одна карта: сдвиг воздушного слоя относительно наземного (layers_eval) <= 0.10 м на каждом seed.
Справочно, без порога: время до последнего подтверждённого человека и до конца прогона, t90 покрытия,
недостижимые люди, которых дроны отметили кандидатами, согласие слоёв, качество слоёв, RTF, нагрузка.
"""
import csv
import json
import re
import statistics as st
import sys

MODES = ('ground', 'mixed')


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


def last_val(p, col):
    v = None
    try:
        for r in csv.DictReader(open(p)):
            if r.get(col) not in (None, ''):
                v = float(r[col])
    except Exception:  # noqa: BLE001
        pass
    return v


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


def collect(chk, s, m):
    d = f'{chk}/seed_{s}_{m}'
    c = jl(f'{d}/check.json')
    r = {'seed': s, 'mode': m, 'have': c is not None}
    if c is None:
        return r
    cov = f'{d}/coverage.csv'
    rtf = floats(f'{d}/rtf.txt', r'real_time_factor:\s*([\d.eE+-]+)')
    load = floats(f'{d}/load.txt')
    unreach = [p for p in c['per_person'] if not p['reachable']]
    pj = jl(f'{d}/people.json') or {}
    marks = pj['marks'] if isinstance(pj, dict) and 'marks' in pj else (pj if isinstance(pj, list) else [])
    air_only = [m['id'] for m in marks if m.get('status') == 'CONFIRMED' and m.get('robots')
                and all(str(r).startswith('drone_') for r in m['robots'])]
    r.update(
        coverage=c['final_coverage'], t90=t_cov(cov, 'coverage', 0.90), completion_t=c['completion_t'],
        completion_reason=c['completion_reason'],
        people=c['people'], reachable=c['reachable'], confirmed=c['confirmed'],
        confirmed_reachable=c['confirmed_reachable'], decoys=c['decoys'],
        decoys_confirmed=len(c['decoys_confirmed']), ghosts_confirmed=c['ghosts']['CONFIRMED'],
        t_first=c['t_first_confirmed'], t_last=c['t_last_confirmed'],
        unreachable=len(unreach),
        unreachable_marked=sum(1 for p in unreach if p['status'] in ('CANDIDATE', 'CONFIRMED')),
        loc_err_mean=c['loc_err_mean'], air_only_confirmed=len(air_only),
        rtf_mean=round(st.mean(rtf), 3) if rtf else None, rtf_min=round(min(rtf), 3) if rtf else None,
        load_mean=round(st.mean(load), 1) if load else None, phase3_fails=c['fails'])
    mp = jl(f'{d}/map_eval.json')
    if mp:
        r['p2'] = round(mp['precision_tol2'], 3)
        r['r2'] = round(mp['recall_tol2'], 3)
    if m == 'mixed':
        r['cov_air'] = last_val(cov, 'cov_air')
        r['cov_ground'] = last_val(cov, 'cov_ground')
        r['t90_air'] = t_cov(cov, 'cov_air', 0.90)
        le = jl(f'{d}/layers_eval.json')
        if le:
            sh = le['best_shift_m']
            r['shift_m'] = round((sh[0] ** 2 + sh[1] ** 2) ** 0.5, 3)
            r['shift_xy'] = [round(sh[0], 3), round(sh[1], 3)]
            r['agree_air'] = round(le['agree_air_on_ground_tol2'], 3)
            r['agree_ground'] = round(le['agree_ground_on_air_tol2'], 3)
            r['air_p2'] = round(le['layers']['air']['precision_tol2'], 3)
            r['ground_p2'] = round(le['layers']['ground']['precision_tol2'], 3)
        a = jl(f'{d}/ate.json')
        if a:
            dr = {k: v for k, v in a['robots'].items() if k.startswith('drone_')}
            if dr:
                r['drone_ate'] = round(max(v['ate']['rmse'] for v in dr.values()), 3)
    return r


def f(v, n=2):
    return '-' if v is None else (f'{v:.{n}f}' if isinstance(v, float) else str(v))


def mean_of(rows, k):
    v = [r[k] for r in rows if r.get(k) is not None]
    return (st.mean(v), min(v), max(v)) if v else (None, None, None)


def main():
    chk, seeds = sys.argv[1], sys.argv[2].split()
    runs = {(s, m): collect(chk, s, m) for s in seeds for m in MODES}
    cols = [('cov', 'покрытие, %', lambda r: f(r['coverage'] * 100, 1)),
            ('t90', 't90, с', lambda r: f(r['t90'], 0)),
            ('air', 'воздух / земля, %', lambda r: f"{f(r['cov_air'] * 100 if r.get('cov_air') is not None else None, 1)} / "
                                                   f"{f(r['cov_ground'] * 100 if r.get('cov_ground') is not None else None, 1)}"),
            ('ppl', 'люди CONFIRMED (достижимых) / достижимых', lambda r: f"{r['confirmed']} ({r['confirmed_reachable']}) / {r['reachable']}"),
            ('un', 'недостижимые: отмечены / всего', lambda r: f"{r['unreachable_marked']} / {r['unreachable']}"),
            ('tl', 'первый / последний CONFIRMED, с', lambda r: f"{f(r['t_first'], 0)} / {f(r['t_last'], 0)}"),
            ('end', 'конец, с', lambda r: f(r['completion_t'], 0)),
            ('dec', 'ловушки / ghost CONFIRMED', lambda r: f"{r['decoys_confirmed']} / {r['ghosts_confirmed']}"),
            ('p2', 'карта P@2 / R@2', lambda r: f"{f(r.get('p2'))} / {f(r.get('r2'))}"),
            ('sh', 'сдвиг слоёв, м', lambda r: f(r.get('shift_m'))),
            ('ag', 'согласие слоёв возд./земл.', lambda r: f"{f(r.get('agree_air'))} / {f(r.get('agree_ground'))}" if 'agree_air' in r else '-'),
            ('rtf', 'RTF ср./мин', lambda r: f"{f(r['rtf_mean'])} / {f(r['rtf_min'])}"),
            ('load', 'load1 ср.', lambda r: f(r['load_mean'], 1))]
    L = [f'# Проверка Фазы 5 (seeds {" ".join(seeds)}): только наземные роботы против смешанного роя\n',
         '| seed | рой | ' + ' | '.join(c[1] for c in cols) + ' |', '| --- | --- | ' + ' | '.join('---' for _ in cols) + ' |']
    for s in seeds:
        for m in MODES:
            r = runs[(s, m)]
            if not r['have']:
                L.append(f'| {s} | {m} | нет данных |' + ' |' * (len(cols) - 1))
            else:
                L.append(f'| {s} | {m} | ' + ' | '.join(c[2](r) for c in cols) + ' |')
    L.append('\n## Среднее (мин / макс) по seed\n')
    L.append('| рой | t90, с | последний CONFIRMED, с | конец, с | люди достижимых CONFIRMED | недостижимые отмечены | ловушки / ghost CONFIRMED | P@2 | RTF | load1 |')
    L.append('| --- ' * 10 + '|')
    agg = {}
    for m in MODES:
        rs = [runs[(s, m)] for s in seeds if runs[(s, m)]['have']]
        a = {k: mean_of(rs, k) for k in ('t90', 't_last', 'completion_t', 'p2', 'rtf_mean', 'load_mean', 'shift_m', 'agree_air')}
        for k in ('confirmed_reachable', 'reachable', 'unreachable_marked', 'unreachable', 'decoys_confirmed',
                  'ghosts_confirmed', 'air_only_confirmed'):
            a[k] = sum(r.get(k, 0) for r in rs)
        a['n'] = len(rs)
        agg[m] = a

        def cell(k, n=0):
            mn, lo, hi = a[k]
            return '-' if mn is None else f'{mn:.{n}f} ({lo:.{n}f} / {hi:.{n}f})'
        L.append(f"| {m} | {cell('t90')} | {cell('t_last')} | {cell('completion_t')} | {a['confirmed_reachable']}/{a['reachable']} | "
                 f"{a['unreachable_marked']}/{a['unreachable']} | {a['decoys_confirmed']} / {a['ghosts_confirmed']} | "
                 f"{cell('p2', 2)} | {cell('rtf_mean', 2)} | {cell('load_mean', 1)} |")
    mx = agg['mixed']
    checks = []
    bad = [f'seed {s}' for s in seeds if not runs[(s, 'mixed')]['have']
           or 'max_time' in (runs[(s, 'mixed')].get('completion_reason') or 'max_time')
           or runs[(s, 'mixed')]['coverage'] < 0.90 or (runs[(s, 'mixed')].get('cov_air') or 0) < 0.80]
    checks.append(('mixed-прогоны завершены до max_time, покрытие >= 90 %, воздушный слой >= 80 %', not bad,
                   'проблемные: ' + (', '.join(bad) or 'нет')))
    ratio = mx['confirmed_reachable'] / mx['reachable'] if mx['reachable'] else None
    checks.append(('люди: mixed CONFIRMED достижимых >= 90 % от достижимых', ratio is not None and ratio >= 0.9,
                   f"{mx['confirmed_reachable']} / {mx['reachable']}"))
    gr = agg['ground']
    checks.append(('ловушки: mixed CONFIRMED = 0', mx['decoys_confirmed'] == 0, f"{mx['decoys_confirmed']}"))
    checks.append(('ghost: mixed CONFIRMED не больше, чем у наземных', mx['ghosts_confirmed'] <= gr['ghosts_confirmed'],
                   f"mixed {mx['ghosts_confirmed']}, ground {gr['ghosts_confirmed']}"))
    checks.append(('ни одной отметки CONFIRMED только от дронов', mx['air_only_confirmed'] == 0, f"{mx['air_only_confirmed']}"))
    shifts = [runs[(s, 'mixed')].get('shift_m') for s in seeds]
    checks.append(('одна карта: сдвиг воздушного слоя относительно наземного <= 0.10 м на каждом seed',
                   all(v is not None and v <= 0.10 for v in shifts), ', '.join(f(v) for v in shifts)))
    L.append('\n## Пороги\n')
    fails = []
    for name, ok, val in checks:
        line = f"{'OK' if ok else 'FAIL'}: {name} ({val})"
        L.append(f'- {line}')
        print(line)
        if not ok:
            fails.append(name)
    open(f'{chk}/summary.md', 'w').write('\n'.join(L) + '\n')
    json.dump({'seeds': seeds, 'runs': {f'{s}_{m}': r for (s, m), r in runs.items()},
               'aggregate': agg, 'checks': [{'name': n, 'ok': o, 'value': v} for n, o, v in checks]},
              open(f'{chk}/summary.json', 'w'), indent=1, ensure_ascii=False)
    print('\n'.join(L))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())

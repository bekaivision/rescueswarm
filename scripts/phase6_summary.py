#!/usr/bin/env python3
"""Сводка проверки Фазы 6: поиск людей смешанным роем при разных уровнях радиосвязи (хост, stdlib).

  python3 scripts/phase6_summary.py <check_dir> "<seeds>" "<levels>"

Читает в <check_dir>/seed_<S>_<уровень>/ файлы check.json (phase3_eval), map_eval.json, comms_eval.json,
rtf.txt, load.txt, retries.txt, и в <check_dir>/outage_seed_<S>/ comms_eval.json (тест обрыва связи).
Пишет <check_dir>/summary.md и summary.json, печатает строки OK/FAIL. Код возврата 1, если есть FAIL.

Пороги (заданы до прогонов):
  1. прогоны с уровнями ideal и good завершены (EXPLORATION COMPLETE); medium и poor только отмечаются;
  2. ideal: покрытие у базы >= 90 %, CONFIRMED достижимых >= 90 % от достижимых, ловушек CONFIRMED 0;
  3. на каждом уровне: ловушек CONFIRMED 0;
  4. на каждом уровне: итоговое покрытие у базы >= идеального - 2 % (данные к концу дошли до базы);
  5. тест обрыва связи: outage_ok (пропускается с пометкой, если прогона нет).
"""
import csv
import glob
import json
import os
import re
import statistics as st
import sys

REF5 = 'results/phase-5/check'


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


def collect(d, s, lvl):
    c = jl(f'{d}/check.json')
    r = {'seed': s, 'level': lvl, 'have': c is not None}
    if c is None:
        return r
    rtf = floats(f'{d}/rtf.txt', r'real_time_factor:\s*([\d.eE+-]+)')
    load = floats(f'{d}/load.txt')
    r.update(coverage=c['final_coverage'], completion_t=c['completion_t'], completion_reason=c['completion_reason'],
             reachable=c['reachable'], confirmed=c['confirmed'], confirmed_reachable=c['confirmed_reachable'],
             decoys_confirmed=len(c['decoys_confirmed']), ghosts_confirmed=c['ghosts']['CONFIRMED'],
             rtf_mean=round(st.mean(rtf), 3) if rtf else None,
             load_mean=round(st.mean(load), 1) if load else None)
    mp = jl(f'{d}/map_eval.json')
    if mp:
        r['p2'] = round(mp['precision_tol2'], 3)
    ce = jl(f'{d}/comms_eval.json')
    if ce:
        r['cov_base'] = ce.get('coverage_base')
        r['cov_coll'] = ce.get('coverage_collective')
        r['t90_base'] = ce.get('t90_base')
        r['t90_coll'] = ce.get('t90_collective')
        r['end_t'] = ce.get('duration')
        tr = ce.get('traffic') or {}
        r['kbs'] = round(tr['tx_bytes_per_s_mean'] / 1000, 2) if tr.get('tx_bytes_per_s_mean') is not None else None
        cn = ce.get('connectivity') or {}
        r['pairs_up'] = cn.get('pairs_up_mean')
        r['path_base'] = cn.get('path_to_base_mean')
        sy = ((ce.get('map_sync') or {}).get('pooled')) or {}
        r['sync_med'], r['sync_p90'] = sy.get('median'), sy.get('p90')
        r['det_med'] = (ce.get('detection_delay') or {}).get('median')
    else:
        r['cov_base'] = c['final_coverage']
        r['t90_base'] = t_cov(f'{d}/coverage.csv', 'coverage', 0.90)
    r['finished'] = os.path.exists(f'{d}/launch.log') and 'EXPLORATION COMPLETE' in open(f'{d}/launch.log', errors='ignore').read()
    return r


def f(v, n=2):
    return '-' if v is None else (f'{v:.{n}f}' if isinstance(v, float) else str(v))


def pct(v, n=1):
    return '-' if v is None else f'{v * 100:.{n}f}'


def stat_cell(vals, n=1, scale=1.0):
    v = [x * scale for x in vals if x is not None]
    return '-' if not v else f'{st.mean(v):.{n}f} ({min(v):.{n}f} / {max(v):.{n}f})'


def main():
    chk, seeds, levels = sys.argv[1], sys.argv[2].split(), sys.argv[3].split()
    runs = {(s, l): collect(f'{chk}/seed_{s}_{l}', s, l) for s in seeds for l in levels}
    cols = [('базовое покрытие, %', lambda r: pct(r.get('cov_base'))),
            ('идеальное, %', lambda r: pct(r.get('cov_coll'))),
            ('t90 база / идеал., с', lambda r: f"{f(r.get('t90_base'), 0)} / {f(r.get('t90_coll'), 0)}"),
            ('CONFIRMED достижимых / достижимых', lambda r: f"{r['confirmed_reachable']} / {r['reachable']}"),
            ('ловушки / ghost CONFIRMED', lambda r: f"{r['decoys_confirmed']} / {r['ghosts_confirmed']}"),
            ('P@2', lambda r: f(r.get('p2'))),
            ('трафик, кБ/с на узел', lambda r: f(r.get('kbs'))),
            ('пар на связи, %', lambda r: pct(r.get('pairs_up'), 0)),
            ('путь до базы, %', lambda r: pct(r.get('path_base'), 0)),
            ('задержка карты мед. / p90, с', lambda r: f"{f(r.get('sync_med'), 1)} / {f(r.get('sync_p90'), 1)}"),
            ('задержка обнаружений мед., с', lambda r: f(r.get('det_med'), 1)),
            ('конец, с', lambda r: f(r.get('completion_t'), 0)),
            ('RTF ср.', lambda r: f(r.get('rtf_mean')))]
    L = [f'# Проверка Фазы 6 (seeds {" ".join(seeds)}): поиск людей при разных уровнях радиосвязи\n',
         '| seed | связь | ' + ' | '.join(c[0] for c in cols) + ' |', '| --- | --- | ' + ' | '.join('---' for _ in cols) + ' |']
    for s in seeds:
        for l in levels:
            r = runs[(s, l)]
            if not r['have']:
                L.append(f'| {s} | {l} | нет данных |' + ' |' * (len(cols) - 1))
            else:
                L.append(f'| {s} | {l} | ' + ' | '.join(c[1](r) for c in cols) + ' |')

    # сводка по уровням
    L.append('\n## Среднее (мин / макс) по seed\n')
    keys = [('cov_base', 'покрытие у базы, %', 100, 1), ('cov_coll', 'идеальное, %', 100, 1), ('t90_base', 't90 база, с', 1, 0),
            ('kbs', 'трафик, кБ/с', 1, 2), ('pairs_up', 'пар на связи, %', 100, 0), ('path_base', 'путь до базы, %', 100, 0),
            ('sync_med', 'задержка карты мед., с', 1, 1), ('sync_p90', 'p90, с', 1, 1), ('det_med', 'задержка обнаружений, с', 1, 1),
            ('completion_t', 'конец, с', 1, 0), ('p2', 'P@2', 1, 2)]
    L.append('| связь | ' + ' | '.join(k[1] for k in keys) + ' | CONFIRMED достижимых | ловушки / ghost |')
    L.append('| --- ' * (len(keys) + 3) + '|')
    agg = {}
    for l in levels:
        rs = [runs[(s, l)] for s in seeds if runs[(s, l)]['have']]
        a = {k: sum(r.get(k, 0) for r in rs) for k in ('confirmed_reachable', 'reachable', 'decoys_confirmed', 'ghosts_confirmed')}
        a['n'] = len(rs)
        agg[l] = a
        L.append(f'| {l} | ' + ' | '.join(stat_cell([r.get(k[0]) for r in rs], k[3], k[2]) for k in keys) +
                 f" | {a['confirmed_reachable']}/{a['reachable']} | {a['decoys_confirmed']} / {a['ghosts_confirmed']} |")
    # справочная строка Фазы 5: прямая связь
    ref = []
    for s in seeds:
        d = f'{os.path.dirname(os.path.abspath(chk.rstrip("/")))}/../phase-5/check/seed_{s}_mixed'
        d = os.path.normpath(d)
        if os.path.isdir(d):
            ref.append(collect(d, s, 'ref'))
    ref = [r for r in ref if r['have']]
    if ref:
        for r in ref:
            r['cov_base'] = r['coverage']
            r['t90_base'] = t_cov(f"{chk}/../../phase-5/check/seed_{r['seed']}_mixed/coverage.csv", 'coverage', 0.90)
        a = {k: sum(r.get(k, 0) for r in ref) for k in ('confirmed_reachable', 'reachable', 'decoys_confirmed', 'ghosts_confirmed')}
        L.append('| прямая связь (Фаза 5) | ' + ' | '.join(stat_cell([r.get(k[0]) for r in ref], k[3], k[2]) for k in keys) +
                 f" | {a['confirmed_reachable']}/{a['reachable']} | {a['decoys_confirmed']} / {a['ghosts_confirmed']} |")

    # тест обрыва
    outd = sorted(glob.glob(f'{chk}/outage_seed_*'))
    L.append('\n## Тест обрыва связи\n')
    out_checks = []
    if not outd:
        L.append('Прогона с обрывом нет (OUTAGE=0 или он не запускался).')
    for d in outd:
        ce = jl(f'{d}/comms_eval.json')
        L.append(f'Каталог: `{os.path.basename(d)}`\n')
        if not ce or not ce.get('outages'):
            L.append('Нет данных comms_eval.json или в нём нет обрывов.')
            out_checks.append((False, f'{os.path.basename(d)}: нет данных'))
            continue
        L.append('| узел | обрыв, с | путь за обрыв, м | своя карта v0 -> v1 | у базы v0 -> v1 | досинхронизация, с | обнаружений за обрыв / дошло позже | outage_ok |')
        L.append('| --- ' * 8 + '|')
        for o in ce['outages']:
            L.append(f"| {o['node']} | {o['t0']:.0f}-{o['t1']:.0f} | {f(o['moved_m'])} | {f(o['own_v0'], 0)} -> {f(o['own_v1'], 0)} | "
                     f"{f(o['base_v0'], 0)} -> {f(o['base_v1'], 0)} | {f(o['resync_s'], 1)} | "
                     f"{o['dets_in_outage']} / {o['dets_arrived_after']} | {'да' if o['outage_ok'] else 'нет'} |")
            out_checks.append((o['outage_ok'], f"{os.path.basename(d)} {o['node']}"))
        L.append('\nКритерии: робот двигался (>= 2 м); своя карта росла (>= 0.5 версии в секунду); у базы за обрыв версий почти не прибавилось (<= 3); '
                 'после возврата связи база догнала за <= 60 с.')

    # пороги
    checks = []
    for l in ('ideal', 'good'):
        if l in levels:
            bad = [f'seed {s}' for s in seeds if not runs[(s, l)]['have'] or not runs[(s, l)].get('finished')]
            checks.append((f'прогоны с уровнем {l} завершены (EXPLORATION COMPLETE)', not bad, 'проблемные: ' + (', '.join(bad) or 'нет')))
    for l in levels:
        if l in ('medium', 'poor'):
            fin = [s for s in seeds if runs[(s, l)].get('finished')]
            L.append(f'\nСправочно: уровень {l}, завершено {len(fin)} из {len(seeds)} прогонов.')
    if 'ideal' in levels:
        rs = [runs[(s, 'ideal')] for s in seeds if runs[(s, 'ideal')]['have']]
        low = [f"seed {r['seed']}" for r in rs if (r.get('cov_base') or 0) < 0.90]
        checks.append(('ideal: покрытие у базы >= 90 %', not low and bool(rs), 'ниже: ' + (', '.join(low) or 'нет')))
        a = agg['ideal']
        ratio = a['confirmed_reachable'] / a['reachable'] if a['reachable'] else None
        checks.append(('ideal: CONFIRMED достижимых >= 90 % от достижимых', ratio is not None and ratio >= 0.9,
                       f"{a['confirmed_reachable']} / {a['reachable']}"))
    dec = {l: agg[l]['decoys_confirmed'] for l in levels}
    checks.append(('каждый уровень: ловушек CONFIRMED = 0', all(v == 0 for v in dec.values()),
                   ', '.join(f'{l}: {v}' for l, v in dec.items())))
    gap = [f"{r['level']} seed {r['seed']}" for r in runs.values()
           if r['have'] and r.get('cov_base') is not None and r.get('cov_coll') is not None and r['cov_base'] < r['cov_coll'] - 0.02]
    checks.append(('каждый уровень: покрытие у базы >= идеального - 2 % к концу', not gap, 'не дошло: ' + (', '.join(gap) or 'нет')))
    if out_checks:
        bad = [n for ok, n in out_checks if not ok]
        checks.append(('тест обрыва связи: outage_ok', not bad, 'не прошли: ' + (', '.join(bad) or 'нет')))
    else:
        L.append('\n(Порог обрыва связи пропущен: прогона нет.)')

    L.append('\n## Пороги\n')
    fails = []
    for name, ok, val in checks:
        line = f"{'OK' if ok else 'FAIL'}: {name} ({val})"
        L.append(f'- {line}')
        print(line)
        if not ok:
            fails.append(name)
    open(f'{chk}/summary.md', 'w').write('\n'.join(L) + '\n')
    json.dump({'seeds': seeds, 'levels': levels, 'runs': {f'{s}_{l}': r for (s, l), r in runs.items()}, 'aggregate': agg,
               'checks': [{'name': n, 'ok': o, 'value': v} for n, o, v in checks]},
              open(f'{chk}/summary.json', 'w'), indent=1, ensure_ascii=False)
    print('\n'.join(L))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""Фаза 2: оценка одного прогона check-phase2 (запускается на хосте, только стандартная библиотека).

  phase2_eval.py SEED_DIR SEED MAX_SIM_S N DETERMINISM_OK(0|1) RETRIES
Читает coverage.csv, launch.log, safety.json, procstat.jsonl, rtf.txt в SEED_DIR, пишет check.json
и печатает строки FAIL:/OK: (код выхода 1, если были FAIL).
"""
import csv
import json
import os
import re
import statistics as st
import sys


def main():
    d, seed, max_s, n, det, retries = sys.argv[1], sys.argv[2], float(sys.argv[3]), int(sys.argv[4]), sys.argv[5] == '1', int(sys.argv[6])
    robots = [f'ground_{i}' for i in range(1, n + 1)]
    fails = []
    res = {'seed': int(seed), 'retries': retries}

    rows = list(csv.DictReader(open(f'{d}/coverage.csv'))) if os.path.exists(f'{d}/coverage.csv') else []
    cov = [(float(r['sim_time_s']), float(r['coverage'])) for r in rows]
    res['final_coverage'] = cov[-1][0:2][1] if cov else 0.0
    res['final_t'] = cov[-1][0] if cov else 0.0
    for p in (0.5, 0.8, 0.9, 0.95):
        res[f't{int(p * 100)}'] = next((t for t, c in cov if c >= p), None)
    res['dist'] = {r: float(rows[-1][f'dist_{r}']) for r in robots} if rows else {}
    t90 = res['t90']
    if t90 is None or t90 > max_s:
        fails.append(f'seed {seed}: покрытие 90 % не достигнуто за {max_s:.0f} с (итог {res["final_coverage"]:.1%})')
    for r in robots:
        if res['dist'].get(r, 0) <= 5.0:
            fails.append(f'seed {seed}: {r} проехал {res["dist"].get(r, 0):.1f} м (нужно > 5)')

    log = open(f'{d}/launch.log', errors='replace').read().splitlines() if os.path.exists(f'{d}/launch.log') else []
    goals = {r: 0 for r in robots}
    auctions = {r: 0 for r in robots}
    unstick = {r: 0 for r in robots}
    black = {r: 0 for r in robots}
    reached = {r: 0 for r in robots}
    yields = {r: 0 for r in robots}    # уступил дорогу роботу с меньшим номером
    waits = {r: 0 for r in robots}     # ждал, пока уступит сосед
    temp_fail = {r: 0 for r in robots}  # неудача цели рядом с роботом (временный чёрный список)
    complete_line = None
    for i, l in enumerate(log):
        m = re.search(r'\[(ground_\d+)\.explorer\]: (.*)', l)
        if m and m[1] in goals:
            r, msg = m[1], m[2]
            if re.match(r'цель #\d+: \(', msg):
                goals[r] += 1
            elif msg.startswith('аукцион #'):
                auctions[r] += 1
            elif msg.startswith('застряли'):
                unstick[r] += 1
            elif msg.startswith('чёрный список'):
                if 'временно' in msg:
                    temp_fail[r] += 1
                else:
                    black[r] += 1
            elif msg.startswith('заблокирован роботом'):
                yields[r] += 1
            elif msg.startswith('жду: робот'):
                waits[r] += 1
            elif re.match(r'цель #\d+ достигнута', msg):
                reached[r] += 1
        if 'EXPLORATION COMPLETE' in l and complete_line is None:
            complete_line = i
    res.update(goals=goals, auction_rounds=auctions, unstick=unstick, blacklist=black, goals_reached=reached,
               yields=yields, waits=waits, temp_fail=temp_fail)
    for r in robots:
        if goals[r] < 1:
            fails.append(f'seed {seed}: в логе аукциона нет целей робота {r}')
    # смерти процессов: до и после COMPLETE
    deaths = [(i, l) for i, l in enumerate(log) if 'process has died' in l]
    before = [l for i, l in deaths if complete_line is None or i < complete_line]
    after = [l for i, l in deaths if complete_line is not None and i > complete_line]
    res['deaths_before_complete'] = len(before)
    res['deaths_after_complete'] = len(after)
    res['deaths_after_complete_detail'] = sorted({re.sub(r'.*\[ERROR\] (\[[^\]]+\]).*exit code (-?\d+).*', r'\1 exit \2', l) for l in after})
    if before:
        fails.append(f'seed {seed}: процесс упал до EXPLORATION COMPLETE: {before[0][:120]}')

    sf = f'{d}/safety.json'
    if os.path.exists(sf):
        s = json.load(open(sf))
        res['safety'] = s
        if s['min_robot_distance_m'] < 0.6:
            fails.append(f'seed {seed}: мин. расстояние между роботами {s["min_robot_distance_m"]:.2f} м < 0.6')
        # цепочка: controller/behaviors/explorer -> cmd_vel_nav -> collision_monitor -> cmd_vel
        for r in robots:
            for topic, key, allowed in (('cmd_vel', 'cmd_vel_publishers', 'collision_monitor'),
                                        ('cmd_vel_nav', 'cmd_vel_nav_publishers', 'controller_server|behavior_server|explorer')):
                pubs = s.get(key, {}).get(r, [])
                for pub in pubs:
                    if not re.fullmatch(rf'/{r}/({allowed})', pub):
                        fails.append(f'seed {seed}: у /{r}/{topic} посторонний издатель {pub}')
                if not pubs:
                    fails.append(f'seed {seed}: монитор не увидел издателей /{r}/{topic}')
    else:
        fails.append(f'seed {seed}: нет safety.json')

    if not det:
        fails.append(f'seed {seed}: генератор недетерминирован (sha256 world.sdf различается)')
    res['deterministic'] = det

    rtf = []
    if os.path.exists(f'{d}/rtf.txt'):
        rtf = [float(x) for x in re.findall(r'real_time_factor: ([\d.]+)', open(f'{d}/rtf.txt').read())]
    res['rtf_mean'] = round(st.mean(rtf), 3) if rtf else None
    res['rtf_min'] = round(min(rtf), 3) if rtf else None
    res['rtf_n'] = len(rtf)
    ps = []
    if os.path.exists(f'{d}/procstat.jsonl'):
        ps = [json.loads(l) for l in open(f'{d}/procstat.jsonl') if l.strip()]
        ps = [p for p in ps if p['total_cores'] > 0.5 and p['t'] > 20]
    if ps:
        gs = sorted({g for p in ps for g in p['groups']})
        res['cpu_cores_total_mean'] = round(st.mean(p['total_cores'] for p in ps), 2)
        res['cpu_groups_mean'] = {g: round(st.mean(p['groups'].get(g, 0.0) for p in ps), 2) for g in gs}
        res['rss_mib_mean'] = round(st.mean(p['rss_mib'] for p in ps))
    res['fails'] = fails
    json.dump(res, open(f'{d}/check.json', 'w'), indent=1, ensure_ascii=False)
    for f in fails:
        print('FAIL:', f)
    if not fails:
        print(f'OK: seed {seed}: покрытие {res["final_coverage"]:.1%}, t90={t90}')
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()

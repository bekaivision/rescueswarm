#!/usr/bin/env python3
"""Фаза 3: оценка поиска людей (запускается на хосте, только стандартная библиотека).

  phase3_eval.py SEED_DIR SEED MAX_SIM_S          оценка одного прогона -> SEED_DIR/check.json
  phase3_eval.py --summary CHK_DIR "1 2 3 7"      сводка по seed -> CHK_DIR/summary.md (+ суммарный recall)

В SEED_DIR: people.json (отметки), gt.json (эталон: people, decoys), coverage.csv, launch.log.
Сопоставление отметок с эталоном: жадно по ближайшим парам, порог MATCH_M, один к одному.
Критерии (FAIL): ложный CONFIRMED на ловушке, отвергнут достижимый человек, нет EXPLORATION COMPLETE
до max_time, процесс упал до завершения; в сводке: recall CONFIRMED достижимых людей >= 90 % по всем seed.
"""
import csv
import json
import math
import os
import re
import statistics as st
import sys

MATCH_M = 1.0
MIN_RECALL = 0.90


def match(marks, objs):
    """Жадное сопоставление: [(dist, mark_idx, obj_idx)] по возрастанию, один к одному."""
    pairs = sorted((math.hypot(m['x'] - o['x'], m['y'] - o['y']), i, j)
                   for i, m in enumerate(marks) for j, o in enumerate(objs))
    used_m, used_o, out = set(), set(), {}
    for d, i, j in pairs:
        if d > MATCH_M:
            break
        if i in used_m or j in used_o:
            continue
        used_m.add(i)
        used_o.add(j)
        out[j] = (i, d)
    return out   # obj_idx -> (mark_idx, dist)


def confirmed_time(m):
    ts = [h['t'] for h in m.get('history', []) if h['status'] == 'CONFIRMED']
    return min(ts) if ts else None


def evaluate(d, seed, max_s):
    fails = []
    res = {'seed': int(seed)}
    if not os.path.exists(f'{d}/gt.json'):
        print(f'FAIL: seed {seed}: нет gt.json (прогон не стартовал?)')
        json.dump({'seed': int(seed), 'fails': ['no gt.json']}, open(f'{d}/check.json', 'w'))
        return 1
    gt = json.load(open(f'{d}/gt.json'))
    people, decoys = gt['people'], gt['decoys']
    marks = json.load(open(f'{d}/people.json'))['marks'] if os.path.exists(f'{d}/people.json') else []
    if not os.path.exists(f'{d}/people.json'):
        fails.append(f'seed {seed}: нет people.json')
    objs = people + decoys
    mt = match(marks, objs)
    np_ = len(people)
    matched_marks = {i for i, _ in mt.values()}

    per_person, err = [], []
    for j, p in enumerate(people):
        st_ = marks[mt[j][0]]['status'] if j in mt else 'MISSED'
        per_person.append(dict(id=p['id'], reachable=bool(p['reachable']), status=st_,
                               err=round(mt[j][1], 3) if j in mt else None,
                               mark=marks[mt[j][0]]['id'] if j in mt else None,
                               reason=marks[mt[j][0]].get('reason') if j in mt else None,
                               t_confirmed=confirmed_time(marks[mt[j][0]]) if j in mt else None))
        if st_ == 'CONFIRMED':
            err.append(mt[j][1])
    per_decoy = []
    for k, c in enumerate(decoys):
        j = np_ + k
        st_ = marks[mt[j][0]]['status'] if j in mt else 'NOT_MARKED'
        per_decoy.append(dict(id=c['id'], type=c['type'], status=st_,
                              reason=marks[mt[j][0]].get('reason') if j in mt else None))
    ghosts = {'CANDIDATE': 0, 'CONFIRMED': 0, 'REJECTED': 0}
    ghost_list = []
    for i, m in enumerate(marks):
        if i not in matched_marks:
            ghosts[m['status']] += 1
            ghost_list.append(dict(id=m['id'], status=m['status'], x=m['x'], y=m['y'], reason=m.get('reason')))
    reach = [p for p in per_person if p['reachable']]
    n_reach = len(reach)
    conf_all = sum(p['status'] == 'CONFIRMED' for p in per_person)
    conf_reach = sum(p['status'] == 'CONFIRMED' for p in reach)
    cc_all = sum(p['status'] in ('CONFIRMED', 'CANDIDATE') for p in per_person)
    cc_reach = sum(p['status'] in ('CONFIRMED', 'CANDIDATE') for p in reach)
    n_conf_marks = sum(m['status'] == 'CONFIRMED' for m in marks)
    decoys_conf = [c['id'] for c in per_decoy if c['status'] == 'CONFIRMED']
    persons_rej = [p['id'] for p in per_person if p['status'] == 'REJECTED']
    reach_rej = [p['id'] for p in reach if p['status'] == 'REJECTED']
    tc = sorted(p['t_confirmed'] for p in per_person if p['t_confirmed'] is not None)
    unresolved = [m['id'] for m in marks if m['status'] == 'CANDIDATE']
    res.update(
        people=np_, reachable=n_reach, decoys=len(decoys),
        confirmed=conf_all, confirmed_reachable=conf_reach,
        recall_confirmed=conf_all / np_ if np_ else None,
        recall_confirmed_reachable=conf_reach / n_reach if n_reach else None,
        recall_cc=cc_all / np_ if np_ else None, recall_cc_reachable=cc_reach / n_reach if n_reach else None,
        precision_confirmed=(sum(1 for j, (i, _) in mt.items() if j < np_ and marks[i]['status'] == 'CONFIRMED') / n_conf_marks
                             if n_conf_marks else None),
        decoys_rejected=sum(c['status'] == 'REJECTED' for c in per_decoy), decoys_confirmed=decoys_conf,
        persons_rejected=persons_rej, persons_rejected_reachable=reach_rej,
        persons_candidate=sum(p['status'] == 'CANDIDATE' for p in per_person),
        persons_missed=sum(p['status'] == 'MISSED' for p in per_person),
        ghosts=ghosts, ghost_list=ghost_list,
        loc_err_mean=round(st.mean(err), 3) if err else None, loc_err_max=round(max(err), 3) if err else None,
        t_first_confirmed=tc[0] if tc else None, t_last_confirmed=tc[-1] if tc else None,
        unresolved_candidates=unresolved, per_person=per_person, per_decoy=per_decoy, n_marks=len(marks))

    rows = list(csv.DictReader(open(f'{d}/coverage.csv'))) if os.path.exists(f'{d}/coverage.csv') else []
    res['final_coverage'] = float(rows[-1]['coverage']) if rows else 0.0
    log = open(f'{d}/launch.log', errors='replace').read().splitlines() if os.path.exists(f'{d}/launch.log') else []
    comp = next((l for l in log if 'EXPLORATION COMPLETE' in l), None)
    sc = next((l for l in log if 'SEARCH COMPLETE' in l), None)
    m = re.search(r'EXPLORATION COMPLETE t=([\d.]+)', comp or '')
    res['completion_t'] = float(m[1]) if m else None
    res['completion_reason'] = re.search(r'\((.*)\)', comp).group(1) if comp and re.search(r'\((.*)\)', comp) else None
    m = re.search(r'SEARCH COMPLETE t=([\d.]+)', sc or '')
    res['search_complete_t'] = float(m[1]) if m else None
    res['gave_up_log'] = sorted(set(re.findall(r'кандидат (mark_\d+) оставлен', '\n'.join(log))))
    res['inspect_goals'] = sum(1 for l in log if re.search(r'цель #\d+: .* осмотр mark_', l))
    deaths = [i for i, l in enumerate(log) if 'process has died' in l]
    ci = next((i for i, l in enumerate(log) if 'EXPLORATION COMPLETE' in l), None)
    if any(ci is None or i < ci for i in deaths):
        fails.append(f'seed {seed}: процесс упал до EXPLORATION COMPLETE')
    if res['completion_t'] is None or 'max_time' in (res['completion_reason'] or '') or res['completion_t'] >= max_s:
        fails.append(f'seed {seed}: прогон не завершён до max_time ({max_s:.0f} с): {res["completion_reason"]}')
    if res['final_coverage'] < 0.90:
        fails.append(f'seed {seed}: покрытие {res["final_coverage"]:.1%} < 90 %')
    if decoys_conf:
        fails.append(f'seed {seed}: ловушки ложно CONFIRMED: {decoys_conf}')
    if reach_rej:
        fails.append(f'seed {seed}: достижимые люди REJECTED: {reach_rej}')
    res['fails'] = fails
    json.dump(res, open(f'{d}/check.json', 'w'), indent=1, ensure_ascii=False)
    for f in fails:
        print('FAIL:', f)
    f1 = lambda v: '-' if v is None else f'{v:.1f}'  # noqa: E731
    if not fails:
        print(f'OK: seed {seed}: confirmed {conf_all}/{np_} (достижимых {conf_reach}/{n_reach}), '
              f'ловушек отвергнуто {res["decoys_rejected"]}/{len(decoys)}, ghost {sum(ghosts.values())}, '
              f't_last={f1(res["t_last_confirmed"])}, завершено {f1(res["completion_t"])}')
    return 1 if fails else 0


def summary(chk, seeds):
    L = [f'# Проверка Фазы 3 (seeds {" ".join(seeds)})\n',
         '| seed | людей / достижимых | CONFIRMED (достижимых) | CANDIDATE | REJECTED (люди) | ловушек отвергнуто / ложно CONFIRMED | ghost C/K/R | ошибка позиции ср./макс, м | t первого / последнего, с | осмотров | оставлено | покрытие | завершено, с | итог |',
         '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    tot_conf = tot_reach = 0
    fl = []
    f1 = lambda v: '-' if v is None else f'{v:.1f}'  # noqa: E731
    for s in seeds:
        try:
            c = json.load(open(f'{chk}/seed_{s}/check.json'))
        except Exception:
            L.append(f'| {s} | нет данных |' + ' |' * 12)
            fl.append(f'seed {s}: нет check.json')
            continue
        tot_conf += c['confirmed_reachable']
        tot_reach += c['reachable']
        g = c['ghosts']
        L.append(f"| {s} | {c['people']} / {c['reachable']} | {c['confirmed']} ({c['confirmed_reachable']}) | {c['persons_candidate']} | "
                 f"{len(c['persons_rejected'])} | {c['decoys_rejected']}/{c['decoys']} / {len(c['decoys_confirmed'])} | "
                 f"{g['CANDIDATE']}/{g['CONFIRMED']}/{g['REJECTED']} | {c['loc_err_mean']} / {c['loc_err_max']} | "
                 f"{f1(c['t_first_confirmed'])} / {f1(c['t_last_confirmed'])} | {c['inspect_goals']} | {len(c['gave_up_log'])} | "
                 f"{c['final_coverage']:.1%} | {f1(c['completion_t'])} | {'OK' if not c['fails'] else 'FAIL'} |")
        fl += c['fails']
    rec = tot_conf / tot_reach if tot_reach else 0.0
    L.append(f'\nRecall CONFIRMED достижимых людей по всем seed: {tot_conf}/{tot_reach} = {rec:.1%} (порог {MIN_RECALL:.0%})')
    if rec < MIN_RECALL:
        fl.append(f'recall CONFIRMED достижимых {rec:.1%} < {MIN_RECALL:.0%}')
    L.append('\nОшибки:' if fl else '\nОшибок нет.')
    L += [f'- {x}' for x in fl]
    open(f'{chk}/summary.md', 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))
    return 1 if fl else 0


if __name__ == '__main__':
    if sys.argv[1] == '--summary':
        sys.exit(summary(sys.argv[2], sys.argv[3].split()))
    sys.exit(evaluate(sys.argv[1], sys.argv[2], float(sys.argv[3])))

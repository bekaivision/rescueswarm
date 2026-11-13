#!/usr/bin/env python3
"""Офлайн-повтор слияния отметок на журналах прогонов (хост, python3 + numpy, без ROS).

  replay_fusion.py RUN_DIR [RUN_DIR ...]      повтор текущего кода, таблица "до/после" по каждому каталогу
  replay_fusion.py --root results             все каталоги с detections.jsonl под results/
  replay_fusion.py --validate RUN_DIR ...     только сверка: повтор БАЗОВОГО кода против people.json прогона

Что читается из каталога прогона:
  detections.jsonl      термо-детекции (x, y, range, temp, h, len) - те же поля, что получает Fusion.update
  rgb_detections.jsonl  RGB-рамки (bearing, half_w, score); поза робота берётся из pose_log.csv
  pose_log.csv          позы роботов 5 Гц (source=pose - то, что видит people_fusion; нет - gt): виды для
                        отрицательного свидетельства (observe_view) и привязка RGB-рамок
  final_map.npy/.yaml   карта для прямой видимости (итоговая, не на момент кадра - приближение)
  gt.json, people.json  эталон и фактический результат прогона (сверка и оценка)
Каталоги без pose_log.csv (Фаза 2-3 до журнала поз) повторяются без видов: not_seen_close и RGB не
воспроизводятся, сверка с people.json по ним приблизительная (в таблице помечены '*').

Базовый код (до правок): `git show HEAD:src/rs_perception/rs_perception/marks.py`, либо --old FILE.
Оценка - тем же сопоставлением, что scripts/phase3_eval.py (match, MATCH_M = 1.0 м).
"""
import argparse
import csv
import importlib.util
import json
import math
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MARKS_PATH = os.path.join(ROOT, 'src', 'rs_perception', 'rs_perception', 'marks.py')
sys.path.insert(0, HERE)
import phase3_eval as ev  # noqa: E402


def load_marks(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def baseline_module(old):
    """Базовая версия marks.py: файл из --old или HEAD репозитория."""
    if old is None:
        old = '/tmp/rs_marks_baseline.py'
        src = subprocess.run(['git', '-C', ROOT, 'show', 'HEAD:src/rs_perception/rs_perception/marks.py'],
                             capture_output=True, text=True, check=True).stdout
        with open(old, 'w') as f:
            f.write(src)
    return load_marks(old, 'marks_baseline')


class World:
    """Карта для прямой видимости (как PeopleFusion._los) и позы роботов."""

    def __init__(self, d):
        self.map = None
        p = os.path.join(d, 'final_map.npy')
        if os.path.exists(p) and os.path.exists(os.path.join(d, 'final_map.yaml')):
            data = np.load(p).astype(np.int16)
            yml = open(os.path.join(d, 'final_map.yaml')).read().split()
            # image: ..\norigin:\n- ox\n- oy\n- 0.0\nresolution: r\nrow0: bottom
            ox = float(yml[yml.index('origin:') + 2])
            oy = float(yml[yml.index('origin:') + 4])
            res = float(yml[yml.index('resolution:') + 1])
            if 'row0: top' in open(os.path.join(d, 'final_map.yaml')).read():
                data = data[::-1]
            self.map = (data, res, ox, oy)
        self.poses = {}
        pl = os.path.join(d, 'pose_log.csv')
        self.has_poses = os.path.exists(pl)
        if self.has_poses:
            rows = list(csv.DictReader(open(pl)))
            srcs = {(r['robot'], r['source']) for r in rows}
            for r in rows:
                rid = r['robot']
                if rid.startswith('drone'):
                    continue
                if (rid, 'pose') in srcs and r['source'] != 'pose':
                    continue
                if (rid, 'pose') not in srcs and r['source'] != 'gt':
                    continue
                self.poses.setdefault(rid, []).append((float(r['t']), float(r['x']), float(r['y']), float(r['yaw'])))

    def pose_at(self, rid, t):
        a = self.poses.get(rid)
        if not a:
            return None
        lo, hi = 0, len(a) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if a[mid][0] < t:
                lo = mid + 1
            else:
                hi = mid
        best = a[lo]
        if lo > 0 and abs(a[lo - 1][0] - t) < abs(best[0] - t):
            best = a[lo - 1]
        return best[1:]

    def los(self, x0, y0, x1, y1):
        if self.map is None:
            return True
        data, res, ox, oy = self.map
        n = int(max(abs(x1 - x0), abs(y1 - y0)) / res) + 2
        xs = np.linspace(x0, x1, n)
        ys = np.linspace(y0, y1, n)
        i = np.floor((xs - ox) / res).astype(int)
        j = np.floor((ys - oy) / res).astype(int)
        h, w = data.shape
        if ((i < 0) | (i >= w) | (j < 0) | (j >= h)).any():
            return False
        occ = data[j, i] >= 65
        occ &= np.hypot(xs - x1, ys - y1) > 0.45
        return not occ.any()


def read_jsonl(p):
    out = []
    if os.path.exists(p):
        for l in open(p):
            try:
                out.append(json.loads(l))
            except ValueError:
                pass
    return out


def replay(mod, d, params=None):
    """Повтор одного прогона. Возвращает список отметок в формате people.json."""
    world = World(d)
    dets = read_jsonl(os.path.join(d, 'detections.jsonl'))
    rgbs = read_jsonl(os.path.join(d, 'rgb_detections.jsonl'))
    air = sorted({x['robot'] for x in dets if x['robot'].startswith('drone')})
    fu = mod.Fusion(params, air)
    ev_ = []
    # приоритет при равном времени: вид (0) раньше детекции (1) раньше RGB (2)
    for rid, arr in world.poses.items():
        for (t, x, y, yaw) in arr:
            ev_.append((t, 0, rid, (x, y, yaw)))
    nan = float('nan')
    for x in dets:
        ev_.append((x['t'], 1, x['robot'], x))
    for x in rgbs:
        ev_.append((x['t'], 2, x['robot'], x))
    ev_.sort(key=lambda e: (e[0], e[1]))
    for t, kind, rid, x in ev_:
        if kind == 0:
            fu.observe_view(rid, x[0], x[1], x[2], t, world.los)
        elif kind == 1:
            fu.update(x['x'], x['y'], rid, x['range'], x['temp'],
                      nan if x['h'] is None else x['h'], nan if x['len'] is None else x['len'], t)
        else:
            pose = world.pose_at(rid, t)
            if pose is not None:
                fu.rgb_observe(rid, pose[0], pose[1], pose[2], x['bearing'], x['half_w'], x['score'], t, world.los)
    out = []
    for m in fu.reported():
        out.append(dict(id=m.id, status=mod.NAMES[m.status], x=round(m.x, 3), y=round(m.y, 3),
                        n_obs=m.n_obs, n_close=m.n_close, reason=m.reason,
                        height_m=None if math.isnan(m.height()) else round(m.height(), 3),
                        history=[dict(t=t, status=s, reason=r) for t, s, r in m.history]))
    return out, world.has_poses


def score(marks, gt):
    """Те же критерии, что в phase3_eval.evaluate (основная метрика, 1 м до центра)."""
    people, decoys = gt['people'], gt['decoys']
    objs = people + decoys
    mt = ev.match(marks, objs)
    np_ = len(people)
    matched = {i for i, _ in mt.values()}
    conf = [p['id'] for j, p in enumerate(people)
            if p['reachable'] and j in mt and marks[mt[j][0]]['status'] == 'CONFIRMED']
    rej = [p['id'] for j, p in enumerate(people)
           if p['reachable'] and j in mt and marks[mt[j][0]]['status'] == 'REJECTED']
    traps = [c['id'] for k, c in enumerate(decoys)
             if np_ + k in mt and marks[mt[np_ + k][0]]['status'] == 'CONFIRMED']
    ghost_c = sum(1 for i, m in enumerate(marks) if i not in matched and m['status'] == 'CONFIRMED')
    ghost_all = sum(1 for i, m in enumerate(marks) if i not in matched)
    opened = sum(1 for m in marks if m['status'] == 'CANDIDATE')
    # недостающий достижимый человек с открытым кандидатом рядом (<= 1.5 м): живой прогон пошлёт осмотр
    pend = sum(1 for j, p in enumerate(people) if p['reachable'] and not (j in mt and marks[mt[j][0]]['status'] == 'CONFIRMED')
               and any(m['status'] == 'CANDIDATE' and math.hypot(m['x'] - p['x'], m['y'] - p['y']) <= 1.5 for m in marks))
    return dict(conf=conf, n_reach=sum(1 for p in people if p['reachable']), rej=rej, traps=traps,
                ghost_c=ghost_c, ghost=ghost_all, cand=opened, pend=pend)


def conf_set(marks, gt):
    """Множество (id человека/ловушки) -> статус отметки: для сверки с фактом."""
    objs = gt['people'] + gt['decoys']
    mt = ev.match(marks, objs)
    return {objs[j]['id']: marks[i]['status'] for j, (i, _) in mt.items()}


def find_runs(root):
    out = []
    for r, _, f in os.walk(root):
        if 'detections.jsonl' in f:
            out.append(r)
    return sorted(out)


_MODS = {}


def work(args):
    """Один каталог: повтор базового и нового кода (для пула процессов)."""
    d, params, old_path = args
    if 'old' not in _MODS:
        _MODS['old'] = baseline_module(old_path)
        _MODS['new'] = load_marks(MARKS_PATH, 'marks_new')
    gtp = os.path.join(d, 'gt.json')
    if not os.path.exists(gtp):
        return None
    gt = json.load(open(gtp))
    mo, has = replay(_MODS['old'], d)
    mn, _ = replay(_MODS['new'], d, params)
    fm = None
    pp = os.path.join(d, 'people.json')
    if os.path.exists(pp):
        fm = json.load(open(pp))['marks']
    return d, gt, mo, mn, has, fm


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dirs', nargs='*')
    ap.add_argument('--root', help='искать каталоги с detections.jsonl под этим путём')
    ap.add_argument('--old', help='файл базовой версии marks.py (по умолчанию HEAD)')
    ap.add_argument('--verbose', action='store_true')
    ap.add_argument('--set', action='append', default=[], metavar='K=V', help='параметр Fusion для нового кода (число)')
    ap.add_argument('--quiet', action='store_true', help='печатать только итог и регрессии')
    ap.add_argument('-j', type=int, default=4, help='число процессов')
    a = ap.parse_args()
    params = {k: float(v) for k, v in (kv.split('=') for kv in a.set)}
    dirs = list(a.dirs) + (find_runs(a.root) if a.root else [])
    baseline_module(a.old)    # один раз выгрузить HEAD во временный файл
    old_path = a.old or '/tmp/rs_marks_baseline.py'
    if a.j > 1 and len(dirs) > 1:
        import multiprocessing
        with multiprocessing.Pool(a.j) as pool:
            results = pool.map(work, [(d, params, old_path) for d in dirs])
    else:
        results = [work((d, params, old_path)) for d in dirs]
    print(f'{"run":58s} {"reach":>5s} | {"conf old/new":>12s} | {"trap o/n":>8s} | {"ghostC o/n":>10s} | {"cand o/n":>8s} | {"pend o/n":>8s} | valid')
    tot = dict(po=0, pn=0, co=0, cn=0, r=0, to=0, tn=0, go=0, gn=0)
    bad = []
    for res in results:
        if res is None:
            continue
        d, gt, mo, mn, has, fm = res
        so, sn = score(mo, gt), score(mn, gt)
        valid = '-'
        if fm is not None:
            sf = score(fm, gt)
            same = set(sf['conf']) == set(so['conf']) and set(sf['traps']) == set(so['traps'])
            cnt = lambda ms: tuple(sum(m['status'] == s for m in ms) for s in ('CONFIRMED', 'CANDIDATE', 'REJECTED'))  # noqa: E731
            valid = 'ok' if same and sf['ghost_c'] == so['ghost_c'] else \
                f'DIFF fact={len(sf["conf"])}/{sf["n_reach"]} trap{len(sf["traps"])} gC{sf["ghost_c"]}'
            valid += f' [C/K/R fact={cnt(fm)} replay={cnt(mo)}]' if cnt(fm) != cnt(mo) else ''
        name = os.path.relpath(d, 'results') if d.startswith('results') else d
        name = name + ('' if has else '*')
        if not a.quiet:
            print(f'{name[:58]:58s} {so["n_reach"]:5d} | {len(so["conf"]):5d}/{len(sn["conf"]):<6d} | '
                  f'{len(so["traps"]):3d}/{len(sn["traps"]):<4d} | {so["ghost_c"]:4d}/{sn["ghost_c"]:<5d} | '
                  f'{so["cand"]:3d}/{sn["cand"]:<4d} | {so["pend"]:3d}/{sn["pend"]:<4d} | {valid}')
        lost = sorted(set(so['conf']) - set(sn['conf']))
        gain = sorted(set(sn['conf']) - set(so['conf']))
        worse = (len(sn['traps']) > len(so['traps']) or set(sn['rej']) - set(so['rej'])
                 or sn['ghost_c'] > so['ghost_c'])
        if lost or gain or worse or a.verbose:
            print(f'    {name}: lost={lost} gained={gain} traps old/new={so["traps"]}/{sn["traps"]} '
                  f'rej old/new={so["rej"]}/{sn["rej"]} ghostC {so["ghost_c"]}/{sn["ghost_c"]}')
        tot['po'] += so['pend']
        tot['pn'] += sn['pend']
        tot['co'] += len(so['conf'])
        tot['cn'] += len(sn['conf'])
        tot['r'] += so['n_reach']
        tot['to'] += len(so['traps'])
        tot['tn'] += len(sn['traps'])
        tot['go'] += so['ghost_c']
        tot['gn'] += sn['ghost_c']
        if lost or worse:
            bad.append(name)
    print(f'ИТОГО reach={tot["r"]} conf old/new={tot["co"]}/{tot["cn"]} traps {tot["to"]}/{tot["tn"]} '
          f'ghostC {tot["go"]}/{tot["gn"]} pending-open {tot["po"]}/{tot["pn"]}')
    if bad:
        print('РЕГРЕССИЯ:', ', '.join(bad))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""Оценка прогона Фазы 6 (радиосвязь): покрытие у базы и «идеальное», трафик, связность, задержки (хост, stdlib + numpy).

  python3 scripts/phase6_eval.py <каталог прогона>

Читает: coverage.csv (последний столбец coverage_collective), detections.jsonl (поле rx), pose_log.csv,
comms/comms_params.json, comms/links.csv, comms/traffic.csv, comms/channels.csv, comms/agent_<узел>.csv.
Пишет <каталог>/comms_eval.json и короткий текст в stdout. Нет файла или столбца: значение null.
"""
import csv
import json
import math
import os
import sys
from collections import defaultdict

import numpy as np


def rows(p):
    try:
        return list(csv.DictReader(open(p)))
    except Exception:  # noqa: BLE001
        return None


def fl(v):
    try:
        if v in (None, ''):
            return None
        return float(v)
    except Exception:  # noqa: BLE001
        return None


def jl(p):
    try:
        return json.load(open(p))
    except Exception:  # noqa: BLE001
        return None


def stats(v):
    """Медиана, p90, максимум."""
    v = [x for x in v if x is not None]
    if not v:
        return {'n': 0, 'median': None, 'p90': None, 'max': None}
    a = np.array(v, float)
    return {'n': len(v), 'median': round(float(np.median(a)), 3),
            'p90': round(float(np.percentile(a, 90)), 3), 'max': round(float(a.max()), 3)}


def first_t(ts, vals, thr, t_min=None):
    """Первое t, где vals >= thr (и t >= t_min)."""
    for t, x in zip(ts, vals):
        if t_min is not None and t < t_min:
            continue
        if x >= thr:
            return t
    return None


def coverage(d, out):
    cv = rows(f'{d}/coverage.csv')
    if not cv:
        return
    tcol = 'sim_time_s' if 'sim_time_s' in cv[0] else list(cv[0].keys())[0]
    ts = [fl(r.get(tcol)) for r in cv]
    base = [fl(r.get('coverage')) for r in cv]
    col = [fl(r.get('coverage_collective')) for r in cv]
    ok = [i for i in range(len(cv)) if ts[i] is not None]
    if not ok:
        return
    out['duration'] = ts[ok[-1]]
    bt = [(ts[i], base[i]) for i in ok if base[i] is not None]
    ct = [(ts[i], col[i]) for i in ok if col[i] is not None]
    out['coverage_base'] = bt[-1][1] if bt else None
    out['coverage_collective'] = ct[-1][1] if ct else None
    out['t90_base'] = first_t([x[0] for x in bt], [x[1] for x in bt], 0.90)
    out['t90_collective'] = first_t([x[0] for x in ct], [x[1] for x in ct], 0.90)
    gap = [col[i] - base[i] for i in ok if base[i] is not None and col[i] is not None]
    out['gap_mean'] = round(float(np.mean(gap)), 4) if gap else None
    out['gap_max'] = round(float(np.max(gap)), 4) if gap else None


def traffic(d, out, duration):
    tr = rows(f'{d}/comms/traffic.csv')
    res = {}
    if tr:
        last = {}
        for r in tr:           # счётчики накопительные: берём последнюю строку узла
            last[r['node']] = r
        dur = duration or max((fl(r.get('t')) or 0) for r in tr) or None
        per = {}
        for n, r in last.items():
            if n == 'base':
                continue
            b = fl(r.get('tx_bytes'))
            per[n] = round(b / dur, 1) if (b is not None and dur) else None
        v = [x for x in per.values() if x is not None]
        res['tx_bytes_per_s'] = per
        res['tx_bytes_per_s_mean'] = round(float(np.mean(v)), 1) if v else None
        tot = lambda k: sum(fl(r.get(k)) or 0 for r in last.values())  # noqa: E731
        res['qdrop_pkts_ratio'] = round(tot('qdrop_pkts') / tot('tx_pkts'), 4) if tot('tx_pkts') else None
        den = tot('rx_pkts') + tot('lost_pkts')
        res['lost_pkts_ratio'] = round(tot('lost_pkts') / den, 4) if den else None
        res['nolink_pkts'] = tot('nolink_pkts')
    ch = rows(f'{d}/comms/channels.csv')
    if ch:
        last = {}
        for r in ch:
            last[r['channel']] = fl(r.get('tx_bytes')) or 0
        s = sum(last.values())
        res['channel_share'] = {k: round(v / s, 4) for k, v in last.items()} if s else {k: 0 for k in last}
        res['channel_bytes'] = last
    out['traffic'] = res or None


def connectivity(d, out):
    lk = rows(f'{d}/comms/links.csv')
    if not lk:
        out['connectivity'] = None
        return
    by_t = defaultdict(list)
    for r in lk:
        by_t[fl(r.get('t'))].append(r)
    up_frac = []
    reach = defaultdict(list)
    nodes = set()
    for r in lk:
        nodes.add(r['a'])
        nodes.add(r['b'])
    robots = sorted(n for n in nodes if n != 'base')
    for t, rs in sorted(by_t.items(), key=lambda x: (x[0] is None, x[0])):
        adj = defaultdict(set)
        ups = 0
        for r in rs:
            u = fl(r.get('up'))
            if u and u >= 0.5:
                ups += 1
                adj[r['a']].add(r['b'])
                adj[r['b']].add(r['a'])
        up_frac.append(ups / len(rs))
        seen, stack = {'base'}, ['base']
        while stack:
            x = stack.pop()
            for y in adj[x]:
                if y not in seen:
                    seen.add(y)
                    stack.append(y)
        for n in robots:
            reach[n].append(1.0 if n in seen else 0.0)
    pb = {n: round(float(np.mean(v)), 4) for n, v in reach.items()}
    out['connectivity'] = {'pairs_up_mean': round(float(np.mean(up_frac)), 4),
                           'path_to_base': pb,
                           'path_to_base_mean': round(float(np.mean(list(pb.values()))), 4) if pb else None}


def load_agent(d, node):
    """agent_<node>.csv -> {source: ([t], [map_have])}."""
    r = rows(f'{d}/comms/agent_{node}.csv')
    if not r:
        return None
    res = defaultdict(lambda: ([], []))
    for x in r:
        t, v = fl(x.get('t')), fl(x.get('map_have'))
        if t is None or v is None:
            continue
        res[x.get('source')][0].append(t)
        res[x.get('source')][1].append(v)
    return res


def sync_delays(d, out):
    params = jl(f'{d}/comms/comms_params.json') or {}
    nodes = [n for n in params.get('nodes', []) if n != 'base']
    if not nodes:       # нет params: узлы по именам файлов
        try:
            nodes = sorted(f[6:-4] for f in os.listdir(f'{d}/comms')
                           if f.startswith('agent_') and f.endswith('.csv') and f != 'agent_base.csv')
        except Exception:  # noqa: BLE001
            nodes = []
    base = load_agent(d, 'base')
    if base is None or not nodes:
        out['map_sync'] = None
        return
    per, pooled, reached, total = {}, [], 0, 0
    for s in nodes:
        own = load_agent(d, s)
        if own is None or s not in own or s not in base:
            per[s] = None
            continue
        ot, ov = own[s]
        bt, bv = base[s]
        vmax = int(max(ov)) if ov else 0
        vs = list(range(1, vmax + 1))
        if len(vs) > 400:
            vs = vs[::max(1, len(vs) // 400)]
        delays, rc = [], 0
        for v in vs:
            ts = first_t(ot, ov, v)
            tb = first_t(bt, bv, v)
            if ts is None:
                continue
            if tb is not None:
                delays.append(tb - ts)
                rc += 1
        n = len(vs)
        per[s] = dict(stats(delays), frac_reached=round(rc / n, 4) if n else None, versions=n)
        pooled += delays
        reached += rc
        total += n
    out['map_sync'] = {'per_source': per, 'pooled': dict(stats(pooled), frac_reached=round(reached / total, 4) if total else None,
                                                          versions=total)}


def detections(d, out):
    delays = []
    dets = []
    try:
        for line in open(f'{d}/detections.jsonl'):
            try:
                j = json.loads(line)
            except Exception:  # noqa: BLE001
                continue
            dets.append(j)
            if j.get('rx') is not None and j.get('t') is not None:
                delays.append(float(j['rx']) - float(j['t']))
    except Exception:  # noqa: BLE001
        out['detection_delay'] = None
        return None
    out['detection_delay'] = stats(delays)
    return dets


def path_len(d, node, t0, t1):
    pl = rows(f'{d}/pose_log.csv')
    if not pl:
        return None
    pts = []
    for r in pl:
        if r.get('robot') != node or (r.get('source') not in (None, 'gt')):
            continue
        t, x, y = fl(r.get('t')), fl(r.get('x')), fl(r.get('y'))
        if None in (t, x, y) or t < t0 or t > t1:
            continue
        pts.append((x, y))
    if len(pts) < 2:
        return 0.0 if pts or pl else None
    return round(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])), 3)


def at(curve, t):
    """Значение накопительной кривой на момент t (последнее значение при t' <= t)."""
    if not curve:
        return None
    ts, vs = curve
    v = None
    for a, b in zip(ts, vs):
        if a <= t:
            v = b
        else:
            break
    return v


def outages(d, out, dets):
    params = jl(f'{d}/comms/comms_params.json') or {}
    outs = params.get('outages') or []
    if not outs:
        out['outages'] = None
        return
    base = load_agent(d, 'base')
    res = []
    for node, t0, t1 in outs:
        t0, t1 = float(t0), float(t1)
        own = load_agent(d, node)
        oc = own.get(node) if own else None
        bc = base.get(node) if base else None
        r = {'node': node, 't0': t0, 't1': t1, 'moved_m': path_len(d, node, t0, t1)}
        r['own_v0'], r['own_v1'] = at(oc, t0), at(oc, t1)
        r['base_v0'], r['base_v1'] = at(bc, t0), at(bc, t1)
        r['resync_s'] = None
        if bc and r['own_v1'] is not None:
            tt = first_t(bc[0], bc[1], r['own_v1'], t_min=t1)
            r['resync_s'] = round(tt - t1, 3) if tt is not None else None
        din = [x for x in (dets or []) if x.get('robot') == node and x.get('t') is not None and t0 <= float(x['t']) <= t1]
        r['dets_in_outage'] = len(din)
        r['dets_arrived_after'] = sum(1 for x in din if x.get('rx') is not None and float(x['rx']) >= t1)
        try:
            r['outage_ok'] = bool(
                r['moved_m'] is not None and r['moved_m'] >= 2.0
                and r['own_v0'] is not None and r['own_v1'] is not None and r['own_v1'] - r['own_v0'] >= 0.5 * (t1 - t0)
                and r['base_v0'] is not None and r['base_v1'] is not None and r['base_v1'] - r['base_v0'] <= 3
                and r['resync_s'] is not None and r['resync_s'] <= 60)
        except Exception:  # noqa: BLE001
            r['outage_ok'] = False
        res.append(r)
    out['outages'] = res


def main():
    d = sys.argv[1].rstrip('/')
    out = {}
    pr = jl(f'{d}/comms/comms_params.json')
    out['level'] = pr.get('level') if pr else None
    coverage(d, out)
    traffic(d, out, out.get('duration'))
    connectivity(d, out)
    sync_delays(d, out)
    dets = detections(d, out)
    outages(d, out, dets)
    json.dump(out, open(f'{d}/comms_eval.json', 'w'), indent=1, ensure_ascii=False)
    print(f"уровень {out['level']}: длительность {out.get('duration')} с, покрытие у базы {out.get('coverage_base')}, "
          f"идеальное {out.get('coverage_collective')}, t90 {out.get('t90_base')} / {out.get('t90_collective')}")
    t = out.get('traffic') or {}
    print(f"трафик: {t.get('tx_bytes_per_s_mean')} Б/с на узел, потери {t.get('lost_pkts_ratio')}, отброшено в очереди {t.get('qdrop_pkts_ratio')}")
    c = out.get('connectivity') or {}
    print(f"связность: пар на связи {c.get('pairs_up_mean')}, путь до базы {c.get('path_to_base_mean')}")
    m = (out.get('map_sync') or {}).get('pooled') or {}
    print(f"задержка карты до базы: медиана {m.get('median')}, p90 {m.get('p90')}, дошло {m.get('frac_reached')}; "
          f"задержка обнаружений: {(out.get('detection_delay') or {}).get('median')}")
    for o in out.get('outages') or []:
        print(f"обрыв {o['node']} {o['t0']:.0f}-{o['t1']:.0f}: {'OK' if o['outage_ok'] else 'FAIL'} "
              f"(путь {o['moved_m']} м, своя {o['own_v0']}->{o['own_v1']}, база {o['base_v0']}->{o['base_v1']}, досинхр. {o['resync_s']} с)")
    return 0


if __name__ == '__main__':
    sys.exit(main())

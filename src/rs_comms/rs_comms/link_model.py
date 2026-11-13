"""Модель радиоканала: дальность, стены, потери фрагментов, пресеты уровней связи.

Чистый python + numpy, без ROS: используется узлом comms_sim и тестами.
"""
import math

import numpy as np

_COMMON = dict(path_exp=2.5, loss_edge=0.5, edge_start=0.6, arq_retries=3,
               frag_bytes=1000)

PRESETS = {
    'ideal': dict(_COMMON, range_m=math.inf, wall_db=0.0, loss_base=0.0,
                  loss_edge=0.0, delay_s=0.0, jitter_s=0.0, bandwidth_Bps=0.0,
                  queue_s=math.inf),
    'good': dict(_COMMON, range_m=40.0, wall_db=3.0, loss_base=0.02,
                 delay_s=0.02, jitter_s=0.01, bandwidth_Bps=250000.0, queue_s=5.0),
    'medium': dict(_COMMON, range_m=20.0, wall_db=5.0, loss_base=0.08,
                   delay_s=0.08, jitter_s=0.04, bandwidth_Bps=60000.0, queue_s=5.0),
    'poor': dict(_COMMON, range_m=12.0, wall_db=8.0, loss_base=0.20,
                 delay_s=0.25, jitter_s=0.15, bandwidth_Bps=15000.0, queue_s=5.0),
}

PARAM_KEYS = ['range_m', 'wall_db', 'path_exp', 'loss_base', 'loss_edge',
              'edge_start', 'arq_retries', 'frag_bytes', 'delay_s', 'jitter_s',
              'bandwidth_Bps', 'queue_s']


class Params(dict):
    """Параметры канала: словарь с доступом по атрибуту (P.range_m)."""

    def __getattr__(self, k):
        try:
            return self[k]
        except KeyError:
            raise AttributeError(k)


def resolve_params(level, overrides=None):
    """Пресет уровня + переопределения. Значение -1.0 или None = взять из пресета."""
    if level not in PRESETS:
        raise ValueError('неизвестный уровень связи: %r' % (level,))
    p = Params(PRESETS[level])
    for k, v in (overrides or {}).items():
        if k not in p or v is None or v == -1.0:
            continue
        p[k] = v
    return p


class Walls:
    """Стены как повёрнутые прямоугольники; векторные массивы для быстрой проверки."""

    def __init__(self, items):
        # items: список (cx, cy, sx, sy, yaw, height)
        a = np.array(items, dtype=float).reshape(-1, 6)
        self.n = len(a)
        self.cx, self.cy = a[:, 0], a[:, 1]
        self.hx, self.hy = a[:, 2] / 2.0, a[:, 3] / 2.0
        self.c, self.s = np.cos(a[:, 4]), np.sin(a[:, 4])
        self.h = a[:, 5]
        # 4 угла (для отладки и визуализации)
        self.corners = []
        for i in range(self.n):
            pts = []
            for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                lx, ly = sx * self.hx[i], sy * self.hy[i]
                pts.append((self.cx[i] + lx * self.c[i] - ly * self.s[i],
                            self.cy[i] + lx * self.s[i] + ly * self.c[i]))
            self.corners.append(pts)

    def __len__(self):
        return self.n


def load_walls(gt):
    """Стены из ground_truth (obstacles с kind == 'wall')."""
    items = []
    for o in (gt or {}).get('obstacles', []) or []:
        if o.get('kind') != 'wall':
            continue
        cx, cy = o['center']
        sx, sy = o['size']
        items.append((cx, cy, sx, sy, float(o.get('yaw', 0.0)),
                      float(o.get('height', 2.0))))
    return Walls(items)


def count_walls(p, q, walls):
    """Сколько стен перекрывает отрезок p->q (x, y, z).

    Стена перекрывает, если отрезок пересекает её прямоугольник на плоскости и
    высота отрезка на участке внутри прямоугольника ниже высоты стены.
    """
    if walls is None or walls.n == 0:
        return 0
    px, py, pz = p
    qx, qy, qz = q
    dx, dy = qx - px, qy - py
    # в локальной системе каждой стены
    ox, oy = px - walls.cx, py - walls.cy
    lx = ox * walls.c + oy * walls.s
    ly = -ox * walls.s + oy * walls.c
    ldx = dx * walls.c + dy * walls.s
    ldy = -dx * walls.s + dy * walls.c
    t0 = np.zeros(walls.n)
    t1 = np.ones(walls.n)
    ok = np.ones(walls.n, dtype=bool)
    for o, d, h in ((lx, ldx, walls.hx), (ly, ldy, walls.hy)):
        par = np.abs(d) < 1e-12
        # параллельно плоскости щели: внутри или нет
        ok &= ~(par & (np.abs(o) > h))
        with np.errstate(divide='ignore', invalid='ignore'):
            ta = (-h - o) / d
            tb = (h - o) / d
        lo = np.where(par, -np.inf, np.minimum(ta, tb))
        hi = np.where(par, np.inf, np.maximum(ta, tb))
        t0 = np.maximum(t0, lo)
        t1 = np.minimum(t1, hi)
    ok &= t0 <= t1
    if not ok.any():
        return 0
    z0 = pz + t0 * (qz - pz)
    z1 = pz + t1 * (qz - pz)
    blocked = ok & (np.minimum(z0, z1) < walls.h)
    return int(blocked.sum())


def link_quality(d, n_walls, P):
    """(связь есть, эффективная дальность, вероятность потери фрагмента)."""
    if math.isinf(P.range_m):
        return True, d, P.loss_base
    d_eff = d * 10 ** (P.wall_db * n_walls / (10.0 * P.path_exp))
    up = d_eff <= P.range_m
    x = (d_eff / P.range_m - P.edge_start) / (1.0 - P.edge_start)
    x = min(max(x, 0.0), 1.0)
    p_frag = P.loss_base + (P.loss_edge - P.loss_base) * x
    return up, d_eff, p_frag


def packet_success_prob(size_bytes, p_frag, P):
    """Вероятность доставки пакета: все фрагменты с учётом повторов канального уровня."""
    k = max(1, math.ceil(size_bytes / P.frag_bytes))
    pf = p_frag ** (1 + int(P.arq_retries))
    return (1.0 - pf) ** k


def parse_outages(s):
    """'ground_1:60:180;drone_2:100:130' (или список таких строк) -> [(узел, t0, t1)]."""
    if not s:
        return []
    parts = []
    for item in ([s] if isinstance(s, str) else list(s)):
        parts += str(item).split(';')
    out = []
    for part in parts:
        part = part.strip()
        if not part:
            continue
        node, t0, t1 = part.split(':')
        out.append((node.strip(), float(t0), float(t1)))
    return out


def in_outage(outages, node, t):
    """Узел сейчас в отключении?"""
    return any(n == node and t0 <= t < t1 for n, t0, t1 in outages)

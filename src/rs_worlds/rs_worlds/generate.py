"""Генератор процедурных миров "разрушенное здание".

Один и тот же seed даёт побайтно одинаковые файлы: только random.Random(seed),
без времени, фиксированный формат чисел.

Запуск:
    ros2 run rs_worlds generate_world --seed 1 --out DIR [--size small|medium]
    python3 -m rs_worlds.generate --seed 1 --out DIR
"""
import argparse
import math
import os
import random

import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy import ndimage

GENERATOR_VERSION = '1.3'
WORLD_NAME = 'rescue'
RES = 0.05            # размер клетки карты, м
ROBOT_RADIUS = 0.30   # радиус диска робота для проходимости, м
WALL_H = 2.0
MARGIN = 3.0          # поле вокруг здания в границах мира, м
MASK_OUTSIDE = 0.0    # маска покрытия: только внутренняя площадь здания
MASK_EXTRA = 0.5      # расширение маски поверх радиуса робота
ENTRANCE_W = 1.6
# Человек: меш Rescue Randy лёжа лицом вверх, головой к +x модели.
# Меш (Collada в см, Gazebo сам применяет unit): Z вверх, ноги в z=0, голова z=1.821,
# руки x=±0.682, лицо в сторону -y (проверено рендером), y=-0.178..+0.179.
# Поворот pose rpy=(-pi/2, 0, -pi/2) (фикс. оси X, затем Z):
#   Rx(-90): (x,y,z) -> (x,z,-y): лицо -y -> вверх, голова +z -> +y;
#   Rz(-90): (x,y) -> (y,-x): голова +y -> +x. Итог: x'=z, y'=-x, z'=-y.
# Бокс после поворота: x' 0..1.821, y' ±0.682, z' -0.179..0.178.
# Сдвиг pose: x' -= 0.9105 (центр тела в начале модели), z' += 0.18 (спина на земле, z>=0.001).
PERSON_MESH_URI = 'model://rescue_randy/meshes/rescue_randy.dae'
PERSON_MESH_POSE = (-0.9105, 0.0, 0.18, -1.5708, 0.0, -1.5708)
PERSON_SIZE = (1.82, 1.36, 0.36)   # габариты меша лёжа (длина, ширина с руками, высота)
# Для зазоров и занятости берём только тело: руки тонкие, полная ширина 1.36 м
# с зазором 0.7 м резко увеличивала число повторов генерации (small: 44 -> 772 за seed 1..20).
PERSON_BODY = (1.82, 0.50)
PERSON_TEX = 'model://rescue_randy/materials/textures/rescue_randy_'
PERSON_TEMP_K = 310.0
# ложные цели: тип -> (температура K, размер (длина, ширина, высота) м, форма)
DECOY_TYPES = {
    'fire': dict(temp_k=600.0, size=(0.4, 0.4, 0.3), shape='cylinder', color=(0.95, 0.45, 0.05)),
    'radiator': dict(temp_k=335.0, size=(0.8, 0.12, 0.6), shape='box', color=(0.85, 0.85, 0.85)),
    'warm_debris': dict(temp_k=309.0, size=(0.7, 0.7, 0.55), shape='box', color=(0.40, 0.33, 0.28)),
}
DECOY_COUNT = {'medium': (3, 6), 'small': (3, 4)}
PEOPLE_DECOY_GAP = 2.0   # минимум между человеком/ложной целью и другими, м
MIN_REACH_FRACTION = 0.70

SIZES = {
    'medium': dict(W=20.0, H=16.0, sw=2.0, nrows=3, mh=3.4, wo=(3.0, 4.2),
                   xs=(-1.5, 1.5), rubble=(15, 40), people=(3, 6)),
    'small': dict(W=12.0, H=10.0, sw=1.8, nrows=2, mh=3.6, wo=(2.2, 2.6),
                  xs=(-0.8, 0.8), rubble=(8, 16), people=(3, 4)),
}


def f4(x):
    return f'{x:.4f}'


# ---------------------------------------------------------------- планировка

def _wall(walls, orient, fixed, a, b, t, doors):
    """Стена вдоль оси orient ('x' или 'y') на координате fixed, от a до b,
    с проёмами doors = [(центр, ширина)]. Концы выступают на t/2 (углы)."""
    cur = a - t / 2.0
    end = b + t / 2.0

    def seg(lo, hi):
        if hi - lo < 0.01:
            return
        mid, ln = (lo + hi) / 2.0, hi - lo
        if orient == 'x':
            walls.append((mid, fixed, ln, t))
        else:
            walls.append((fixed, mid, t, ln))

    for c, w in sorted(doors):
        seg(cur, c - w / 2.0)
        cur = c + w / 2.0
    seg(cur, end)


def make_layout(rng, size):
    p = SIZES[size]
    W, H, sw, nrows = p['W'], p['H'], p['sw'], p['nrows']
    t = rng.uniform(0.15, 0.20)
    xs = rng.uniform(*p['xs'])
    sides = (-1, 1)

    # границы рядов комнат (свои для каждой стороны)
    ybounds = {}
    for s in sides:
        wts = [rng.random() + 0.3 for _ in range(nrows)]
        extra = H - nrows * p['mh']
        ys = [-H / 2.0]
        for k in range(nrows):
            ys.append(ys[-1] + p['mh'] + extra * wts[k] / sum(wts))
        ys[-1] = H / 2.0
        ybounds[s] = ys
    wo = {s: rng.uniform(*p['wo']) for s in sides}
    xspine = {s: xs + s * sw / 2.0 for s in sides}   # линия стены коридора
    xdiv = {s: s * W / 2.0 - s * wo[s] for s in sides}

    # перечень проёмов (без ширины и позиции)
    specs = []
    for s in sides:
        for r in range(nrows):
            specs.append(('spine', s, r))
            specs.append(('div', s, r))
        for r in range(nrows - 1):
            if rng.random() < 0.5:
                specs.append(('row', s, r))
    narrow_idx = set(rng.sample(range(len(specs)), 3))

    doors = []
    for i, (kind, s, r) in enumerate(specs):
        narrow = i in narrow_idx
        w = rng.uniform(0.78, 0.85) if narrow else rng.uniform(1.0, 1.2)
        if kind in ('spine', 'div'):
            lo = ybounds[s][r] + t / 2.0 + 0.3 + w / 2.0
            hi = ybounds[s][r + 1] - t / 2.0 - 0.3 - w / 2.0
            c = rng.uniform(lo, hi)
            x = xspine[s] if kind == 'spine' else xdiv[s]
            doors.append(dict(kind=kind, side=s, row=r, cx=x, cy=c, w=w,
                              orient='v', narrow=narrow))
        else:  # дверь между соседними внутренними комнатами
            xa, xb = sorted((xdiv[s], xspine[s]))
            c = rng.uniform(xa + t / 2.0 + 0.3 + w / 2.0, xb - t / 2.0 - 0.3 - w / 2.0)
            doors.append(dict(kind=kind, side=s, row=r, cx=c, cy=ybounds[s][r + 1],
                              w=w, orient='h', narrow=narrow))
    entrance = dict(kind='entrance', side=0, row=0, cx=xs, cy=-H / 2.0, w=ENTRANCE_W,
                    orient='h', narrow=False)
    doors.append(entrance)

    def dlist(pred, key):
        return [(d[key], d['w']) for d in doors if pred(d)]

    walls = []
    _wall(walls, 'x', -H / 2.0, -W / 2.0, W / 2.0, t,
          dlist(lambda d: d['kind'] == 'entrance', 'cx'))
    _wall(walls, 'x', H / 2.0, -W / 2.0, W / 2.0, t, [])
    _wall(walls, 'y', -W / 2.0, -H / 2.0, H / 2.0, t, [])
    _wall(walls, 'y', W / 2.0, -H / 2.0, H / 2.0, t, [])
    for s in sides:
        _wall(walls, 'y', xspine[s], -H / 2.0, H / 2.0, t,
              dlist(lambda d, s=s: d['kind'] == 'spine' and d['side'] == s, 'cy'))
        _wall(walls, 'y', xdiv[s], -H / 2.0, H / 2.0, t,
              dlist(lambda d, s=s: d['kind'] == 'div' and d['side'] == s, 'cy'))
        for r in range(nrows - 1):
            _wall(walls, 'x', ybounds[s][r + 1], min(s * W / 2.0, xspine[s]),
                  max(s * W / 2.0, xspine[s]), t,
                  dlist(lambda d, s=s, r=r: d['kind'] == 'row' and d['side'] == s
                        and d['row'] == r, 'cx'))

    # зоны
    rooms = []
    for s in sides:
        for r in range(nrows):
            y0, y1 = ybounds[s][r] + t / 2.0, ybounds[s][r + 1] - t / 2.0
            xi0, xi1 = sorted((xdiv[s] - s * t / 2.0, xspine[s] + s * t / 2.0))
            xo0, xo1 = sorted((s * W / 2.0 - s * t / 2.0, xdiv[s] + s * t / 2.0 * s))
            rooms.append(dict(type='room', rect=[xi0, y0, xi1, y1]))
            rooms.append(dict(type='dead_end', rect=[xo0, y0, xo1, y1]))
    corridor = [xs - sw / 2.0 + t / 2.0, -H / 2.0 + t / 2.0,
                xs + sw / 2.0 - t / 2.0, H / 2.0 - t / 2.0]
    narrow = []
    for d in doors:
        if not d['narrow']:
            continue
        a = t / 2.0 + 0.25
        if d['orient'] == 'v':
            narrow.append([d['cx'] - a, d['cy'] - d['w'] / 2.0, d['cx'] + a, d['cy'] + d['w'] / 2.0])
        else:
            narrow.append([d['cx'] - d['w'] / 2.0, d['cy'] - a, d['cx'] + d['w'] / 2.0, d['cy'] + a])
    return dict(size=size, W=W, H=H, t=t, xs=xs, sw=sw, walls=walls, doors=doors,
                rooms=rooms, corridor=corridor, narrow=narrow,
                n_rubble=rng.randint(*p['rubble']), n_people=rng.randint(*p['people']))


# ------------------------------------------------------------------- растр

class Grid:
    def __init__(self, xmin, ymin, xmax, ymax):
        self.xmin, self.ymin, self.xmax, self.ymax = xmin, ymin, xmax, ymax
        self.nx = int(round((xmax - xmin) / RES))
        self.ny = int(round((ymax - ymin) / RES))

    def empty(self):
        return np.zeros((self.ny, self.nx), dtype=bool)

    def cell(self, x, y):
        return (int(math.floor((y - self.ymin) / RES)),
                int(math.floor((x - self.xmin) / RES)))

    def shape_mask(self, cx, cy, sx, sy, yaw, circle=False):
        """Возвращает (j0, j1, i0, i1, маска) для коробки/цилиндра."""
        hd = math.hypot(sx, sy) / 2.0
        i0 = max(0, int(math.floor((cx - hd - self.xmin) / RES)))
        i1 = min(self.nx - 1, int(math.floor((cx + hd - self.xmin) / RES)))
        j0 = max(0, int(math.floor((cy - hd - self.ymin) / RES)))
        j1 = min(self.ny - 1, int(math.floor((cy + hd - self.ymin) / RES)))
        if i1 < i0 or j1 < j0:
            return 0, -1, 0, -1, np.zeros((0, 0), dtype=bool)
        X = self.xmin + (np.arange(i0, i1 + 1) + 0.5) * RES
        Y = self.ymin + (np.arange(j0, j1 + 1) + 0.5) * RES
        X, Y = np.meshgrid(X, Y)
        dx, dy = X - cx, Y - cy
        if circle:
            m = dx * dx + dy * dy <= (sx / 2.0) ** 2
        else:
            c, s = math.cos(yaw), math.sin(yaw)
            m = (np.abs(c * dx + s * dy) <= sx / 2.0) & (np.abs(-s * dx + c * dy) <= sy / 2.0)
        return j0, j1, i0, i1, m

    def stamp(self, grid, *args, **kw):
        j0, j1, i0, i1, m = self.shape_mask(*args, **kw)
        if j1 >= j0:
            grid[j0:j1 + 1, i0:i1 + 1] |= m

    def hits(self, grid, *args, **kw):
        j0, j1, i0, i1, m = self.shape_mask(*args, **kw)
        return j1 >= j0 and bool((grid[j0:j1 + 1, i0:i1 + 1] & m).any())

    def rect_slice(self, rect):
        i0 = max(0, int(math.floor((rect[0] - self.xmin) / RES)))
        i1 = min(self.nx, int(math.ceil((rect[2] - self.xmin) / RES)))
        j0 = max(0, int(math.floor((rect[1] - self.ymin) / RES)))
        j1 = min(self.ny, int(math.ceil((rect[3] - self.ymin) / RES)))
        return slice(j0, j1), slice(i0, i1)


def analyze(g, occ, spawn_poses, limit_rect):
    """Проходимость диском ROBOT_RADIUS из зоны старта и маска покрытия."""
    free = ~occ
    dist = ndimage.distance_transform_edt(free) * RES
    eroded = dist >= ROBOT_RADIUS - 1e-9
    lab, _ = ndimage.label(eroded)
    ids = set()
    for sp in spawn_poses:
        j, i = g.cell(sp['x'], sp['y'])
        if 0 <= j < g.ny and 0 <= i < g.nx and lab[j, i] > 0:
            ids.add(int(lab[j, i]))
        else:
            return None
    reach = np.isin(lab, sorted(ids))
    d_reach = ndimage.distance_transform_edt(~reach) * RES
    reach_free = free & (d_reach <= ROBOT_RADIUS + 1e-9)
    limit = g.empty()
    sj, si = g.rect_slice(limit_rect)
    limit[sj, si] = True
    mask = free & (d_reach <= ROBOT_RADIUS + MASK_EXTRA) & limit
    return dict(free=free, eroded=eroded, reach=reach, reach_free=reach_free, mask=mask)


# ------------------------------------------------------------ наполнение

def gen_rubble(rng, lay, g, wall_occ):
    W, H, xs, sw, t = lay['W'], lay['H'], lay['xs'], lay['sw'], lay['t']
    doors = [d for d in lay['doors'] if d['kind'] != 'entrance' and not d['narrow']]
    items = []
    for _ in range(lay['n_rubble']):
        for _try in range(40):
            mode = rng.random()
            if mode < 0.35 and doors:
                d = rng.choice(doors)
                off = rng.uniform(0.3, 0.7) * rng.choice((-1, 1))
                al = rng.uniform(-d['w'] / 4.0, d['w'] / 4.0)
                if d['orient'] == 'v':
                    cx, cy = d['cx'] + off, d['cy'] + al
                else:
                    cx, cy = d['cx'] + al, d['cy'] + off
            elif mode < 0.55:
                cx = xs + rng.uniform(-(sw / 2.0 - t / 2.0 - 0.1), sw / 2.0 - t / 2.0 - 0.1)
                cy = rng.uniform(-H / 2.0 + 3.0, H / 2.0 - 0.5)
            else:
                r = rng.choice(lay['rooms'])['rect']
                cx = rng.uniform(r[0] + 0.2, r[2] - 0.2)
                cy = rng.uniform(r[1] + 0.2, r[3] - 0.2)
            shape = rng.choice(('box', 'cylinder'))
            if shape == 'box':
                sx, sy = rng.uniform(0.2, 1.0), rng.uniform(0.2, 1.0)
            else:
                sx = sy = rng.uniform(0.2, 1.0)
            yaw = rng.uniform(-math.pi, math.pi)
            h = rng.uniform(0.15, 0.8)
            circ = math.hypot(sx, sy) / 2.0
            j, i = g.cell(cx, cy)
            if wall_occ[j, i]:
                continue
            if abs(cx) > W / 2.0 or abs(cy) > H / 2.0:
                continue
            # вход и зона узких проходов должны оставаться чистыми
            if abs(cx - xs) < 1.0 + circ and cy < -H / 2.0 + 2.5:
                continue
            if any(z[0] - 0.5 - circ < cx < z[2] + 0.5 + circ and
                   z[1] - 0.5 - circ < cy < z[3] + 0.5 + circ for z in lay['narrow']):
                continue
            items.append(dict(shape=shape, cx=cx, cy=cy, sx=sx, sy=sy, yaw=yaw, h=h))
            break
    return items


def place_people(rng, lay, g, occ, an, rooms_reach):
    k = lay['n_people']
    people = []
    pocc = g.empty()
    sx, sy = PERSON_BODY
    reachable_rooms = [i for i, ok in enumerate(rooms_reach) if ok]
    other_rooms = [i for i, ok in enumerate(rooms_reach) if not ok]
    if not reachable_rooms:
        return None
    want_unreach = 1 if (k >= 4 and other_rooms and rng.random() < 0.5) else 0
    for n in range(k):
        pool = other_rooms if n < want_unreach else reachable_rooms
        for _try in range(300):
            ri = rng.choice(pool)
            r = lay['rooms'][ri]['rect']
            cx = rng.uniform(r[0] + 0.5, r[2] - 0.5)
            cy = rng.uniform(r[1] + 0.5, r[3] - 0.5)
            yaw = rng.uniform(-math.pi, math.pi)
            # зазор 0.35 м вокруг тела, чтобы робот мог объехать
            if g.hits(occ, cx, cy, sx + 0.7, sy + 0.7, yaw):
                continue
            if g.hits(pocc, cx, cy, sx + 0.7, sy + 0.7, yaw):
                continue
            g.stamp(pocc, cx, cy, sx + 0.7, sy + 0.7, yaw)
            people.append(dict(x=cx, y=cy, yaw=yaw, room=ri))
            break
        else:
            return None
    return people


def place_decoys(rng, lay, g, occ, rooms_reach, people):
    """Ложные цели (fire, radiator, warm_debris), минимум одна warm_debris.
    Те же зазоры, что у людей; не в узких проходах; >= PEOPLE_DECOY_GAP от людей и друг от друга."""
    n = rng.randint(*DECOY_COUNT[lay['size']])
    types = ['warm_debris'] + [rng.choice(sorted(DECOY_TYPES)) for _ in range(n - 1)]
    rooms = [lay['rooms'][i]['rect'] for i, ok in enumerate(rooms_reach) if ok]
    rooms += [lay['corridor']]
    H = lay['H']
    decoys = []
    for typ in types:
        spec = DECOY_TYPES[typ]
        dl, dw, dh = spec['size']
        for _try in range(400):
            r = rng.choice(rooms)
            yaw = rng.uniform(-math.pi, math.pi)
            if typ == 'radiator':
                side = rng.randrange(4)
                gap = dw / 2.0 + 0.03
                if side < 2:   # у южной/северной стены
                    cx = rng.uniform(r[0] + 0.8, r[2] - 0.8)
                    cy = r[1] + gap if side == 0 else r[3] - gap
                    yaw = 0.0
                else:
                    cy = rng.uniform(r[1] + 0.8, r[3] - 0.8)
                    cx = r[0] + gap if side == 2 else r[2] - gap
                    yaw = math.pi / 2.0
            else:
                cx = rng.uniform(r[0] + 0.6, r[2] - 0.6)
                cy = rng.uniform(r[1] + 0.6, r[3] - 0.6)
            circ = typ == 'fire'
            if cy < -H / 2.0 + 2.5 and r is lay['corridor']:
                continue    # зона входа остаётся чистой
            if any(d['cx'] - 1.0 < cx < d['cx'] + 1.0 and d['cy'] - 1.0 < cy < d['cy'] + 1.0
                   for d in lay['doors']):
                continue
            if any(z[0] - 0.8 < cx < z[2] + 0.8 and z[1] - 0.8 < cy < z[3] + 0.8 for z in lay['narrow']):
                continue
            if typ == 'radiator':
                # у стены: сама стена рядом, зазор 0.35 м только к завалу/людям/другим целям
                if g.hits(occ, cx, cy, dl + 0.3, dw, yaw):
                    continue
            elif g.hits(occ, cx, cy, dl + 0.7, dw + 0.7, yaw, circle=circ):
                continue
            if any(math.hypot(cx - q['x'], cy - q['y']) < PEOPLE_DECOY_GAP for q in people):
                continue
            if any(math.hypot(cx - q['x'], cy - q['y']) < PEOPLE_DECOY_GAP for q in decoys):
                continue
            decoys.append(dict(type=typ, x=cx, y=cy, yaw=yaw, temp_k=spec['temp_k'], size=spec['size'],
                               shape=spec['shape'], color=spec['color']))
            break
        else:
            return None
    return decoys


def _try_build(rng, size, seed):
    lay = make_layout(rng, size)
    W, H, t, xs = lay['W'], lay['H'], lay['t'], lay['xs']
    g = Grid(-W / 2.0 - MARGIN, -H / 2.0 - MARGIN, W / 2.0 + MARGIN, H / 2.0 + MARGIN)
    limit = [-W / 2.0 - MASK_OUTSIDE, -H / 2.0 - MASK_OUTSIDE,
             W / 2.0 + MASK_OUTSIDE, H / 2.0 + MASK_OUTSIDE]
    wall_occ = g.empty()
    for (cx, cy, sx, sy) in lay['walls']:
        g.stamp(wall_occ, cx, cy, sx, sy, 0.0)
    rubble = gen_rubble(rng, lay, g, wall_occ)
    if len(rubble) < SIZES[size]['rubble'][0]:
        return None
    rubble_occ = g.empty()
    for r in rubble:
        g.stamp(rubble_occ, r['cx'], r['cy'], r['sx'], r['sy'], r['yaw'],
                circle=(r['shape'] == 'cylinder'))
    occ0 = wall_occ | rubble_occ
    spawn = [dict(x=xs + dx, y=-H / 2.0 - 0.8, yaw=math.pi / 2.0) for dx in (-2, -1, 0, 1, 2)]
    an0 = analyze(g, occ0, spawn, limit)
    if an0 is None:
        return None
    for z in lay['narrow']:
        if not an0['eroded'][g.rect_slice(z)].any():
            return None
    rooms_reach = []
    for room in lay['rooms']:
        rooms_reach.append(int(an0['reach_free'][g.rect_slice(room['rect'])].sum()) >= 20)
    people = place_people(rng, lay, g, occ0, an0, rooms_reach)
    if people is None:
        return None
    people_occ = g.empty()
    for p in people:
        g.stamp(people_occ, p['x'], p['y'], PERSON_BODY[0], PERSON_BODY[1], p['yaw'])
    # ложные цели ставятся после людей, чтобы раскладка людей не менялась
    decoys = place_decoys(rng, lay, g, occ0 | people_occ, rooms_reach, people)
    if decoys is None:
        return None
    decoy_occ = g.empty()
    for d in decoys:
        g.stamp(decoy_occ, d['x'], d['y'], d['size'][0], d['size'][1], d['yaw'],
                circle=(d['shape'] == 'cylinder'))
    occ = occ0 | people_occ | decoy_occ
    an = analyze(g, occ, spawn, limit)
    if an is None:
        return None
    interior = g.empty()
    isj, isi = g.rect_slice([-W / 2.0 + t / 2.0, -H / 2.0 + t / 2.0,
                             W / 2.0 - t / 2.0, H / 2.0 - t / 2.0])
    interior[isj, isi] = True
    int_free = int((an['free'] & interior).sum())
    int_reach = int((an['reach_free'] & interior).sum())
    if int_reach < MIN_REACH_FRACTION * int_free:
        return None
    return dict(seed=seed, size=size, lay=lay, grid=g, limit=limit, wall_occ=wall_occ,
                rubble_occ=rubble_occ, people_occ=people_occ, decoy_occ=decoy_occ, decoys=decoys, occ=occ, an=an,
                rubble=rubble, people=people, spawn=spawn, interior=interior,
                int_free=int_free, int_reach=int_reach)


def build_world(seed, size='medium'):
    if size not in SIZES:
        raise ValueError('size must be small or medium')
    for attempt in range(200):
        rng = random.Random(int(seed) * 7919 + attempt)
        w = _try_build(rng, size, int(seed))
        if w is not None:
            w['attempt'] = attempt
            return w
    raise RuntimeError(f'не удалось построить мир для seed={seed}')


# ------------------------------------------------------------ ground truth

def _r(x):
    return round(float(x), 4)


def _rect(r):
    return [_r(v) for v in r]


def _zones(w):
    lay, g, an = w['lay'], w['grid'], w['an']
    zones = []
    xs, H, sw, t = lay['xs'], lay['H'], lay['sw'], lay['t']
    spawn_rect = [xs - 2.5, -H / 2.0 - 2.0, xs + 2.5, -H / 2.0 - 0.4]
    zones.append(dict(id='spawn', type='spawn', rect=_rect(spawn_rect), reachable=True))

    def reach(rect, key='reach_free', n=20):
        return int(an[key][g.rect_slice(rect)].sum()) >= n

    zones.append(dict(id='corridor_1', type='corridor', rect=_rect(lay['corridor']),
                      reachable=reach(lay['corridor'])))
    ri = de = 0
    for room in lay['rooms']:
        if room['type'] == 'room':
            ri += 1
            zid = f'room_{ri}'
        else:
            de += 1
            zid = f'dead_end_{de}'
        zones.append(dict(id=zid, type=room['type'], rect=_rect(room['rect']),
                          reachable=reach(room['rect'])))
    for k, z in enumerate(lay['narrow'], 1):
        zones.append(dict(id=f'narrow_{k}', type='narrow_passage', rect=_rect(z),
                          reachable=reach(z, 'eroded', 1) and bool(an['reach'][g.rect_slice(z)].any())))
    return zones


def ground_truth(w):
    lay, g, an = w['lay'], w['grid'], w['an']
    zones = _zones(w)
    room_zone = {}
    ri = de = 0
    for idx, room in enumerate(lay['rooms']):
        if room['type'] == 'room':
            ri += 1
            room_zone[idx] = f'room_{ri}'
        else:
            de += 1
            room_zone[idx] = f'dead_end_{de}'
    obstacles = []
    for k, (cx, cy, sx, sy) in enumerate(lay['walls'], 1):
        obstacles.append(dict(id=f'wall_{k}', kind='wall', shape='box', center=[_r(cx), _r(cy)],
                              size=[_r(sx), _r(sy)], yaw=0.0, height=WALL_H))
    for k, r in enumerate(w['rubble'], 1):
        obstacles.append(dict(id=f'rubble_{k}', kind='rubble', shape=r['shape'],
                              center=[_r(r['cx']), _r(r['cy'])], size=[_r(r['sx']), _r(r['sy'])],
                              yaw=_r(r['yaw']), height=_r(r['h'])))
    people = []
    for k, p in enumerate(w['people'], 1):
        # человек достижим, если рядом с его телом есть клетка, куда доезжает центр робота
        near = False
        j0, j1, i0, i1, m = g.shape_mask(p['x'], p['y'], PERSON_BODY[0] + 1.3,
                                         PERSON_BODY[1] + 1.3, p['yaw'])
        if j1 >= j0:
            near = bool((an['reach'][j0:j1 + 1, i0:i1 + 1] & m).any())
        people.append(dict(id=f'person_{k}', x=_r(p['x']), y=_r(p['y']), yaw=_r(p['yaw']),
                           zone_id=room_zone[p['room']], reachable=near))
    decoys = []
    for k, d in enumerate(w['decoys'], 1):
        zone = next((z['id'] for z in zones if z['type'] in ('room', 'dead_end', 'corridor')
                     and z['rect'][0] <= d['x'] <= z['rect'][2] and z['rect'][1] <= d['y'] <= z['rect'][3]),
                    'unknown')
        j0, j1, i0, i1, m = g.shape_mask(d['x'], d['y'], d['size'][0] + 1.3, d['size'][1] + 1.3, d['yaw'])
        near = bool((an['reach'][j0:j1 + 1, i0:i1 + 1] & m).any()) if j1 >= j0 else False
        decoys.append(dict(id=f'decoy_{k}', type=d['type'], x=_r(d['x']), y=_r(d['y']), yaw=_r(d['yaw']),
                           temp_k=d['temp_k'], size=[_r(v) for v in d['size']], zone_id=zone,
                           reachable=near))
    W, H = lay['W'], lay['H']
    cell = RES * RES
    stats = dict(rooms=sum(1 for z in zones if z['type'] == 'room'),
                 dead_ends=sum(1 for z in zones if z['type'] == 'dead_end'),
                 narrow_passages=sum(1 for z in zones if z['type'] == 'narrow_passage'),
                 rubble=len(w['rubble']), people=len(people), decoys=len(decoys),
                 interior_free_area_m2=_r(w['int_free'] * cell),
                 reachable_area_m2=_r(w['int_reach'] * cell),
                 reachable_fraction=_r(w['int_reach'] / w['int_free']),
                 mask_area_m2=_r(int(an['mask'].sum()) * cell))
    return dict(
        seed=w['seed'], size=w['size'], generator_version=GENERATOR_VERSION,
        generator_attempt=w['attempt'], world_name=WORLD_NAME,
        bounds=dict(xmin=_r(g.xmin), xmax=_r(g.xmax), ymin=_r(g.ymin), ymax=_r(g.ymax)),
        building=dict(xmin=_r(-W / 2.0), xmax=_r(W / 2.0), ymin=_r(-H / 2.0), ymax=_r(H / 2.0),
                      wall_thickness=_r(lay['t']), wall_height=WALL_H),
        map=dict(resolution=RES, free_value=254, occupied_value=0),
        robot_radius=ROBOT_RADIUS, stats=stats, zones=zones, obstacles=obstacles,
        people=people, decoys=decoys,
        spawn_poses=[dict(x=_r(s['x']), y=_r(s['y']), yaw=_r(s['yaw'])) for s in w['spawn']],
    )


# ---------------------------------------------------------------- вывод

def _mat(c):
    s = ' '.join(f4(v) for v in c) + ' 1'
    return f'<material><ambient>{s}</ambient><diffuse>{s}</diffuse></material>'


def world_sdf(w):
    lay = w['lay']
    o = []
    a = o.append
    a('<?xml version="1.0" ?>')
    a('<!-- Сгенерировано rs_worlds, seed=%d, size=%s, версия %s -->' % (w['seed'], w['size'], GENERATOR_VERSION))
    a('<sdf version="1.9">')
    a(f'  <world name="{WORLD_NAME}">')
    a('''    <physics name="2ms" type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <atmosphere type="adiabatic"><temperature>288.15</temperature></atmosphere>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name="v">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.55 0.55 0.5 1</ambient><diffuse>0.55 0.55 0.5 1</diffuse></material>
        </visual>
      </link>
    </model>
''')
    grey = (0.78, 0.78, 0.80)
    a('    <model name="walls">')
    a('      <static>true</static>')
    a('      <link name="link">')
    for k, (cx, cy, sx, sy) in enumerate(lay['walls'], 1):
        size = f'{f4(sx)} {f4(sy)} {f4(WALL_H)}'
        pose = f'{f4(cx)} {f4(cy)} {f4(WALL_H / 2.0)} 0 0 0'
        geo = f'<geometry><box><size>{size}</size></box></geometry>'
        a(f'        <collision name="c{k}"><pose>{pose}</pose>{geo}</collision>')
        a(f'        <visual name="v{k}"><pose>{pose}</pose>{geo}{_mat(grey)}</visual>')
    a('      </link>')
    a('    </model>')
    dark = [(0.45, 0.30, 0.18), (0.36, 0.25, 0.16), (0.28, 0.24, 0.22)]
    for k, r in enumerate(w['rubble'], 1):
        col = dark[k % 3]
        if r['shape'] == 'box':
            geo = f'<geometry><box><size>{f4(r["sx"])} {f4(r["sy"])} {f4(r["h"])}</size></box></geometry>'
        else:
            geo = (f'<geometry><cylinder><radius>{f4(r["sx"] / 2.0)}</radius>'
                   f'<length>{f4(r["h"])}</length></cylinder></geometry>')
        a(f'    <model name="rubble_{k}">')
        a('      <static>true</static>')
        a(f'      <pose>{f4(r["cx"])} {f4(r["cy"])} {f4(r["h"] / 2.0)} 0 0 {f4(r["yaw"])}</pose>')
        a('      <link name="link">')
        a(f'        <collision name="c">{geo}</collision>')
        a(f'        <visual name="v">{geo}{_mat(col)}</visual>')
        a('      </link>')
        a('    </model>')
    geo = f'<geometry><mesh><uri>{PERSON_MESH_URI}</uri></mesh></geometry>'
    ppose = ' '.join(f4(v) for v in PERSON_MESH_POSE)
    pmat = ('<material><diffuse>1 1 1 1</diffuse><specular>1 1 1 1</specular><pbr><metal>'
            f'<albedo_map>{PERSON_TEX}albedo.png</albedo_map>'
            f'<normal_map>{PERSON_TEX}normal.png</normal_map>'
            f'<metalness_map>{PERSON_TEX}metalness.png</metalness_map>'
            f'<roughness_map>{PERSON_TEX}roughness.png</roughness_map>'
            '</metal></pbr></material>')
    for k, p in enumerate(w['people'], 1):
        a(f'    <model name="person_{k}">')
        a('      <static>true</static>')
        a(f'      <pose>{f4(p["x"])} {f4(p["y"])} 0 0 0 {f4(p["yaw"])}</pose>')
        a('      <link name="link">')
        a(f'        <collision name="c"><pose>{ppose}</pose>{geo}</collision>')
        a(f'        <visual name="v"><pose>{ppose}</pose>{geo}{pmat}')
        a('          <plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal">')
        a('            <temperature>%.1f</temperature>' % PERSON_TEMP_K)
        a('          </plugin>')
        a('        </visual>')
        a('      </link>')
        a('    </model>')
    for k, d in enumerate(w['decoys'], 1):
        dl, dw, dh = d['size']
        if d['shape'] == 'box':
            geo = f'<geometry><box><size>{f4(dl)} {f4(dw)} {f4(dh)}</size></box></geometry>'
        else:
            geo = (f'<geometry><cylinder><radius>{f4(dl / 2.0)}</radius>'
                   f'<length>{f4(dh)}</length></cylinder></geometry>')
        a(f'    <model name="decoy_{k}_{d["type"]}">')
        a('      <static>true</static>')
        a(f'      <pose>{f4(d["x"])} {f4(d["y"])} {f4(dh / 2.0)} 0 0 {f4(d["yaw"])}</pose>')
        a('      <link name="link">')
        a(f'        <collision name="c">{geo}</collision>')
        a(f'        <visual name="v">{geo}{_mat(d["color"])}')
        a('          <plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal">')
        a(f'            <temperature>{d["temp_k"]:.1f}</temperature>')
        a('          </plugin>')
        a('        </visual>')
        a('      </link>')
        a('    </model>')
    a('  </world>')
    a('</sdf>')
    return '\n'.join(o) + '\n'


def write_pgm(path, arr_top_down):
    h, wd = arr_top_down.shape
    with open(path, 'wb') as f:
        f.write(f'P5\n{wd} {h}\n255\n'.encode())
        f.write(arr_top_down.astype(np.uint8).tobytes())


def write_map_yaml(path, image, g):
    txt = (f'image: {image}\nmode: trinary\nresolution: {RES}\n'
           f'origin: [{f4(g.xmin)}, {f4(g.ymin)}, 0.0]\nnegate: 0\n'
           f'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    with open(path, 'w') as f:
        f.write(txt)


def preview_image(w, width=800):
    g, an = w['grid'], w['an']
    img = np.zeros((g.ny, g.nx, 3), dtype=np.float64)
    img[:] = (205, 205, 195)
    m = an['mask'][..., None]
    img = np.where(m, img * 0.5 + np.array((120, 220, 120)) * 0.5, img)
    img[w['wall_occ']] = (70, 70, 85)
    img[w['rubble_occ']] = (125, 75, 35)
    img[w['people_occ']] = (230, 160, 140)
    img[w['decoy_occ']] = (240, 150, 40)
    im = Image.fromarray(img[::-1].astype(np.uint8))
    k = width / (g.xmax - g.xmin)
    im = im.resize((width, int(round((g.ymax - g.ymin) * k))), Image.NEAREST)
    d = ImageDraw.Draw(im)

    def px(x, y):
        return ((x - g.xmin) * k, (g.ymax - y) * k)

    for z in w['lay']['narrow']:
        x0, y1 = px(z[0], z[1])
        x1, y0 = px(z[2], z[3])
        d.rectangle([x0 - 2, y0 - 2, x1 + 2, y1 + 2], outline=(255, 140, 0), width=2)
    for p in w['people']:
        x, y = px(p['x'], p['y'])
        d.ellipse([x - 5, y - 5, x + 5, y + 5], fill=(220, 0, 0))
    for q in w['decoys']:
        x, y = px(q['x'], q['y'])
        d.rectangle([x - 5, y - 5, x + 5, y + 5], fill=(255, 140, 0))
    for s in w['spawn']:
        x, y = px(s['x'], s['y'])
        d.ellipse([x - 6, y - 6, x + 6, y + 6], outline=(0, 0, 220), width=2)
        x2, y2 = px(s['x'] + 0.5 * math.cos(s['yaw']), s['y'] + 0.5 * math.sin(s['yaw']))
        d.line([x, y, x2, y2], fill=(0, 0, 220), width=2)
    return im


def write_outputs(w, out):
    os.makedirs(out, exist_ok=True)
    g, an = w['grid'], w['an']
    gt = ground_truth(w)
    with open(os.path.join(out, 'world.sdf'), 'w') as f:
        f.write(world_sdf(w))
    with open(os.path.join(out, 'ground_truth.yaml'), 'w') as f:
        f.write('# Эталон мира (ground truth), сгенерирован rs_worlds\n')
        yaml.safe_dump(gt, f, sort_keys=False, default_flow_style=None, allow_unicode=True)
    gt_img = np.where(w['occ'], 0, 254)[::-1]
    write_pgm(os.path.join(out, 'gt_map.pgm'), gt_img)
    write_map_yaml(os.path.join(out, 'gt_map.yaml'), 'gt_map.pgm', g)
    write_pgm(os.path.join(out, 'coverage_mask.pgm'), np.where(an['mask'], 254, 0)[::-1])
    write_map_yaml(os.path.join(out, 'coverage_mask.yaml'), 'coverage_mask.pgm', g)
    preview_image(w).save(os.path.join(out, 'preview.png'))
    return gt


def main(argv=None):
    ap = argparse.ArgumentParser(description='Генератор мира "разрушенное здание"')
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--size', choices=sorted(SIZES), default='medium')
    a = ap.parse_args(argv)
    w = build_world(a.seed, a.size)
    gt = write_outputs(w, a.out)
    print(f'seed={a.seed} size={a.size} out={a.out} stats={gt["stats"]}')


if __name__ == '__main__':
    main()

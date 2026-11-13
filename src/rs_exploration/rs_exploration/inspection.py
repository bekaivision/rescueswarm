"""Точки осмотра кандидатов-людей (чистый numpy, без ROS).

Точка осмотра = свободная известная клетка в кольце [r_min, r_max] м от кандидата, с запасом от стен
и прямой видимостью на известной карте. Из каждого сектора кольца берётся одна клетка, ближайшая к
r_pref, поэтому у кандидата есть варианты с разных сторон (вид с торца и вид сбоку).
"""
import math

import numpy as np


def line_clear(data, i0, j0, i1, j1, skip_cells=0):
    """Нет занятых/неизвестных клеток на отрезке между клетками (i, j) = (x, y); клетки в skip_cells от (i1, j1) не в счёт."""
    n = int(max(abs(i1 - i0), abs(j1 - j0))) + 1
    if n < 2:
        return True
    xs = np.rint(np.linspace(i0, i1, n)).astype(int)
    ys = np.rint(np.linspace(j0, j1, n)).astype(int)
    h, w = data.shape
    ok = (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)
    if not ok.all():
        return False
    d = np.hypot(xs - i1, ys - j1)
    vals = data[ys, xs]
    bad = (vals >= 65) | (vals < 0)
    bad &= d > skip_cells
    return not bad.any()


def standing_points(an, data, cx, cy, *, r_min=1.0, r_max=2.0, r_pref=1.5, clear_min=0.3,
                    n_sectors=24, skip_m=0.45):
    """Список (x, y, yaw, bearing) точек осмотра кандидата в (cx, cy); yaw смотрит на кандидата."""
    res = an.res
    h, w = data.shape
    ci = int(math.floor((cx - an.ox) / res))
    cj = int(math.floor((cy - an.oy) / res))
    rr = int(math.ceil(r_max / res)) + 1
    i0, i1 = max(ci - rr, 0), min(ci + rr + 1, w)
    j0, j1 = max(cj - rr, 0), min(cj + rr + 1, h)
    if i0 >= i1 or j0 >= j1:
        return []
    jj, ii = np.mgrid[j0:j1, i0:i1]
    dx = (ii + 0.5) * res + an.ox - cx
    dy = (jj + 0.5) * res + an.oy - cy
    dist = np.hypot(dx, dy)
    cand = (dist >= r_min) & (dist <= r_max) & an.known_free[j0:j1, i0:i1] & (an.clear[j0:j1, i0:i1] >= clear_min)
    if not cand.any():
        return []
    ys, xs = np.nonzero(cand)
    ang = np.arctan2(dy[ys, xs], dx[ys, xs])
    sec = ((ang + math.pi) / (2 * math.pi) * n_sectors).astype(int) % n_sectors
    score = np.abs(dist[ys, xs] - r_pref) - 0.3 * np.minimum(an.clear[j0:j1, i0:i1][ys, xs], 0.8)
    order = np.lexsort((score, sec))
    out = []
    last = -1
    skip = int(math.ceil(skip_m / res))
    for k in order:
        if sec[k] == last:
            continue
        # в секторе лучшая клетка может не видеть кандидата: пробуем следующие по порядку
        ii_, jj_ = int(xs[k] + i0), int(ys[k] + j0)
        if line_clear(data, ii_, jj_, ci, cj, skip):
            last = sec[k]
            px, py = (ii_ + 0.5) * res + an.ox, (jj_ + 0.5) * res + an.oy
            out.append((px, py, math.atan2(cy - py, cx - px), math.atan2(py - cy, px - cx)))
    return out


def angle_diff(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


class GiveUp:
    """Ограничение ожидания кандидатов (без ROS). Кандидат оставляем, если
    - к нему нет достижимой точки осмотра дольше noreach_s (за дверью, за стеной), или
    - он открыт дольше max_open_s с первой заявки и сейчас не осматривается.
    Остальной код держит оставленных в gave_up; кандидат остаётся CANDIDATE (оператор его видит)."""

    def __init__(self, noreach_s=60.0, max_open_s=300.0):
        self.noreach_s = noreach_s
        self.max_open_s = max_open_s
        self.since = {}
        self.first = {}

    def update(self, now, open_ids, reachable, goal_key='', blocked=()):
        """Возвращает [(id, причина)] новых оставленных. blocked - ключи на временной паузе."""
        out = []
        for key in open_ids:
            self.first.setdefault(key, now)
            if key in reachable or key == goal_key:
                self.since.pop(key, None)
            elif key not in blocked:
                t0 = self.since.setdefault(key, now)
                if now - t0 >= self.noreach_s:
                    out.append((key, f'нет достижимой точки осмотра {now - t0:.0f} с'))
                    continue
            if key != goal_key and now - self.first[key] >= self.max_open_s:
                out.append((key, f'открыт дольше {self.max_open_s:.0f} с'))
        for key, _ in out:
            self.since.pop(key, None)
        return out


def is_transient_fail(since_sent, min_s):
    """Отказ Nav2 пришёл раньше min_s после отправки цели: это сбой инфраструктуры (TF), а не недостижимая цель."""
    return since_sent < min_s


def count_fail(n, transient):
    """Счётчик неудач осмотра: сбой сразу после отправки цели в счёт не идёт (кулдаун всё равно ставится отдельно)."""
    return n if transient else n + 1


def blocked(block, key, now):
    """Кулдаун на осмотр key ещё идёт (block: id -> время конца)."""
    return now < block.get(key, 0.0)

"""Синхронизация слоёв карты между узлами сети (чистый numpy + zlib, без ROS).

Слой одного источника (робота) хранится как int8 на клетку: -128 клетку не видели,
иначе q = round(лог-шанс * 20). У каждой клетки своя версия (когда она менялась у источника).
Дельта = клетки, изменившиеся после версии base; остальные клетки в дельте = 127 (не менялась).
Здесь же правило «кто кому что присылает» (serve_targets) для ретрансляции данных через соседей.
"""
import zlib

import numpy as np

UNSEEN = -128       # клетку источник не видел
NOCHANGE = 127      # клетка не менялась (только внутри дельты)
SCALE = 20.0        # q = лог-шанс * SCALE
L_MIN = -2.0        # пределы суммы лог-шансов после слияния
L_MAX = 3.5
OCC_THR = 0.4       # лог-шанс от которого клетка считается занятой

ENC_RAW = 0
ENC_ZLIB = 1


class SourceLayer:
    """Слой одного источника: значения, версии клеток и до какой версии слой у нас без пропусков."""

    def __init__(self, n, kind=0):
        self.n = int(n)
        self.kind = int(kind)
        self.q = np.full(self.n, UNSEEN, dtype=np.int8)
        self.ver = np.zeros(self.n, dtype=np.uint32)
        self.have = 0           # непрерывная версия, 0 = ничего нет
        self.changed = False    # что-то поменялось после последнего слияния/публикации

    def apply_full(self, q_new, version):
        """Собственный снимок: клетки, что отличаются, получают версию version."""
        q_new = np.asarray(q_new, dtype=np.int8).reshape(-1)
        mask = q_new != self.q
        if mask.any():
            self.q[mask] = q_new[mask]
            self.ver[mask] = version
            self.changed = True
        self.have = int(version)

    def apply_delta(self, cells, base, version):
        """Чужая дельта. Старые данные не затирают новые. Возвращает число изменённых клеток.

        Если у нас есть всё до base, то слой теперь непрерывен до version; иначе это пропуск:
        значения применяем, но have не растёт (недостающее пришлёт сосед по маяку).
        """
        cells = np.asarray(cells, dtype=np.int8).reshape(-1)
        mask = (cells != NOCHANGE) & (np.uint32(version) > self.ver)
        n = int(mask.sum())
        if n:
            self.q[mask] = cells[mask]
            self.ver[mask] = version
            self.changed = True
        if self.have >= base:
            self.have = max(self.have, int(version))
        return n

    def make_delta(self, base, upto=None):
        """(zlib-байты int8-массива, число изменённых клеток): q где base < ver <= upto, иначе NOCHANGE.

        upto по умолчанию = have: клетки, пришедшие через пропуск (ver > have), не отдаём дальше
        под чужой версией, их отправим, когда пропуск закроется.
        """
        if upto is None:
            upto = self.have
        mask = (self.ver > base) & (self.ver <= upto)
        out = np.where(mask, self.q, np.int8(NOCHANGE)).astype(np.int8)
        return zlib.compress(out.tobytes(), 6), int(mask.sum())  # уровень 6: пустая дельта ~250 Б вместо ~1 КБ


def encode(arr, encoding):
    """int8-массив -> байты для MapLayer.data."""
    raw = np.asarray(arr, dtype=np.int8).tobytes()
    return zlib.compress(raw, 1) if encoding == ENC_ZLIB else raw


def decode(data, encoding):
    """Байты MapLayer.data -> int8-массив."""
    raw = bytes(data)
    if encoding == ENC_ZLIB:
        raw = zlib.decompress(raw)
    return np.frombuffer(raw, dtype=np.int8).copy()


def merge(layers):
    """Слияние слоёв одного рода: L = clip(sum q/20 по источникам, что видели клетку), seen = видел хоть кто-то.

    layers: итерируемое из SourceLayer одинакового размера. Пустой список даёт None.
    """
    layers = list(layers)
    if not layers:
        return None
    n = layers[0].n
    total = np.zeros(n, dtype=np.float32)
    seen = np.zeros(n, dtype=bool)
    for ly in layers:
        s = ly.q != UNSEEN
        total[s] += ly.q[s].astype(np.float32) / SCALE
        seen |= s
    np.clip(total, L_MIN, L_MAX, out=total)
    return total, seen


def to_grid(L, seen):
    """(L, seen) -> int8 для OccupancyGrid: -1 не видели, 100 занято (L >= 0.4), 0 свободно."""
    return np.where(seen, np.where(L >= OCC_THR, 100, 0), -1).astype(np.int8)


def fuse(g_grid, g_seen, a_grid):
    """Общая карта: значение наземных там, где клетку видел хоть один наземный источник, иначе воздушное."""
    return np.where(g_seen, g_grid, a_grid).astype(np.int8)


def serve_targets(me, s, my_have, peers, my_neighbors=None):
    """Кого узел me обслуживает данными источника s.

    my_have: сколько у нас есть от s (версия/seq без пропусков).
    peers: {P: (have_P, neighbors_P)}: по последнему маяку соседа P (свежему): have_P = {источник: версия},
           neighbors_P = множество его соседей.
    my_neighbors: наши соседи сейчас (по умолчанию ключи peers).

    P нуждается, если have_P[s] < my_have. Обслуживаем P, если s == me, либо источник s не сосед P и
    нет третьего узла Y (сосед и P, и нам), у которого данных больше, чем у нас, или поровну, но имя
    Y меньше (тогда это он должен ретранслировать).
    """
    if my_neighbors is None:
        my_neighbors = set(peers)
    out = []
    for p, (have_p, nb_p) in peers.items():
        if p == me or p == s:
            continue
        if have_p.get(s, 0) >= my_have:
            continue
        if s == me:
            out.append(p)
            continue
        if s in nb_p:
            continue    # P слышит источник сам
        better = False
        for y in nb_p:
            if y in (me, p) or y not in my_neighbors or y not in peers:
                continue
            hy = peers[y][0].get(s, 0)
            if hy > my_have or (hy == my_have and y < me):
                better = True
                break
        if not better:
            out.append(p)
    return out

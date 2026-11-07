"""Фронтиры и оценка пути на общей карте (чистый numpy/scipy, без ROS).

Фронтир = известная свободная клетка рядом с (большой) неизвестной областью внутри
области поиска. Малые "дырки" неизвестного (до min_unknown клеток) считаются свободными:
они возникают из-за дискретизации лучей и не должны давать ложных фронтиров.
"""
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

CROSS = ndimage.generate_binary_structure(2, 1)
EIGHT = ndimage.generate_binary_structure(2, 2)


@dataclass
class Analysis:
    res: float
    ox: float
    oy: float
    factor: int
    clear: np.ndarray            # расстояние до ближайшего занятого, м (мелкая сетка)
    known_free: np.ndarray
    frontier: np.ndarray         # маска фронтирных клеток (после фильтра по размеру кластера)
    # кластеры
    cx: np.ndarray = field(default_factory=lambda: np.zeros(0))   # центроид, м
    cy: np.ndarray = field(default_factory=lambda: np.zeros(0))
    tx: np.ndarray = field(default_factory=lambda: np.zeros(0))   # цель (клетка с запасом от стен), м
    ty: np.ndarray = field(default_factory=lambda: np.zeros(0))
    size: np.ndarray = field(default_factory=lambda: np.zeros(0))  # число клеток
    # крупная сетка для оценки пути
    pas_c: np.ndarray = None
    clear_c: np.ndarray = None

    def world(self, ix, iy):
        return self.ox + (np.asarray(ix) + 0.5) * self.res, self.oy + (np.asarray(iy) + 0.5) * self.res


def analyse(data, res, ox, oy, region, *, min_unknown=40, min_cluster=8,
            clear_target=0.3, clear_pass=0.25, factor=4, open_size=7):
    """data: int8 (h, w) карта (-1/0/100); region: [xmin, ymin, xmax, ymax] в мире."""
    h, w = data.shape
    occ = data >= 65
    unk = data < 0
    lab, n = ndimage.label(unk)
    if n:
        sizes = np.bincount(lab.ravel())
        big = sizes >= min_unknown
        big[0] = False
        unk_big = big[lab]
    else:
        unk_big = unk
    known_free = (data == 0)
    passable = ~occ & ~unk_big
    clear = ndimage.distance_transform_edt(~occ).astype(np.float32) * res

    reg = np.zeros((h, w), dtype=bool)
    i0 = max(int(np.floor((region[0] - ox) / res)), 0)
    i1 = min(int(np.ceil((region[2] - ox) / res)), w)
    j0 = max(int(np.floor((region[1] - oy) / res)), 0)
    j1 = min(int(np.ceil((region[3] - oy) / res)), h)
    reg[j0:j1, i0:i1] = True

    # для фронтиров неизвестное должно быть "толстым": сердцевина стены (~0.2 м) неизвестна
    # всегда и не должна давать фронтиров. Открытие убирает полосы тоньше open_size клеток.
    unk_core = ndimage.binary_opening(unk_big, structure=np.ones((open_size, open_size), dtype=bool))
    near_unk = ndimage.binary_dilation(unk_core, structure=CROSS)
    fr = known_free & near_unk & reg
    flab, fn = ndimage.label(fr, structure=EIGHT)
    an = Analysis(res, ox, oy, factor, clear, known_free, np.zeros_like(fr))

    # крупная сетка
    f = factor
    Hc, Wc = -(-h // f), -(-w // f)
    pad = ((0, Hc * f - h), (0, Wc * f - w))
    pas_f = np.pad(passable & (clear >= clear_pass), pad)
    clr_f = np.pad(clear, pad)
    an.pas_c = pas_f.reshape(Hc, f, Wc, f).any(axis=(1, 3))
    an.clear_c = clr_f.reshape(Hc, f, Wc, f).max(axis=(1, 3))

    if fn == 0:
        return an
    fy, fx = np.nonzero(fr)
    fl = flab[fy, fx]
    cnt = np.bincount(fl, minlength=fn + 1)
    keep_lab = cnt >= min_cluster
    keep_lab[0] = False
    if not keep_lab.any():
        return an
    m = keep_lab[fl]
    fy, fx, fl = fy[m], fx[m], fl[m]
    an.frontier[fy, fx] = True
    # центроиды
    labs = np.flatnonzero(keep_lab)
    remap = np.zeros(fn + 1, dtype=np.int64)
    remap[labs] = np.arange(len(labs))
    k = remap[fl]
    nk = len(labs)
    csize = np.bincount(k, minlength=nk).astype(np.float64)
    mx = np.bincount(k, weights=fx, minlength=nk) / csize
    my = np.bincount(k, weights=fy, minlength=nk) / csize
    # цель: клетка кластера с запасом от стен, ближайшая к центроиду
    d2 = (fx - mx[k]) ** 2 + (fy - my[k]) ** 2
    cl = clear[fy, fx]
    thr = np.where(np.bincount(k, weights=(cl >= clear_target + 0.05), minlength=nk) > 0,
                   clear_target + 0.05, clear_target)
    good = cl >= thr[k]
    ok_cluster = np.zeros(nk, dtype=bool)
    ok_cluster[np.unique(k[good])] = True
    gi = np.flatnonzero(good)
    order = gi[np.lexsort((d2[gi], k[gi]))]
    first = np.ones(len(order), dtype=bool)
    first[1:] = k[order][1:] != k[order][:-1]
    sel = order[first]            # по одной клетке на кластер
    tix = np.zeros(nk)
    tiy = np.zeros(nk)
    tix[k[sel]] = fx[sel]
    tiy[k[sel]] = fy[sel]
    cwx, cwy = an.world(mx, my)
    twx, twy = an.world(tix, tiy)
    an.cx, an.cy = cwx[ok_cluster], cwy[ok_cluster]
    an.tx, an.ty = twx[ok_cluster], twy[ok_cluster]
    an.size = csize[ok_cluster]
    return an


def path_costs(an, rx, ry, tx, ty, *, force_radius=0.5):
    """Длина пути (с поправкой на близость стен) от робота до каждой цели; inf = недостижимо."""
    f = an.factor
    cres = an.res * f
    pas = an.pas_c.copy()
    clr = an.clear_c
    Hc, Wc = pas.shape
    # клетка робота и окрестность считаем проходимыми
    rcx = int(np.clip(np.floor((rx - an.ox) / cres), 0, Wc - 1))
    rcy = int(np.clip(np.floor((ry - an.oy) / cres), 0, Hc - 1))
    r = int(np.ceil(force_radius / cres))
    pas[max(rcy - r, 0):rcy + r + 1, max(rcx - r, 0):rcx + r + 1] = True
    ids = -np.ones((Hc, Wc), dtype=np.int64)
    nodes = np.flatnonzero(pas.ravel())
    ids.ravel()[nodes] = np.arange(len(nodes))
    mult = 1.0 + 1.5 * np.clip((0.6 - clr) / 0.35, 0.0, 1.0)

    rows, cols, wts = [], [], []

    def add(sa, sb, length, diag=False):
        a_ok = pas[sa]
        b_ok = pas[sb]
        valid = a_ok & b_ok
        if diag:
            valid &= sdiag1 & sdiag2
        rows.append(ids[sa][valid])
        cols.append(ids[sb][valid])
        wts.append(length * 0.5 * (mult[sa][valid] + mult[sb][valid]))

    # (dy, dx) направления: вправо, вверх, две диагонали
    add((slice(0, Hc), slice(0, Wc - 1)), (slice(0, Hc), slice(1, Wc)), cres)
    add((slice(0, Hc - 1), slice(0, Wc)), (slice(1, Hc), slice(0, Wc)), cres)
    # диагональ (+1,+1): углы (y,x+1) и (y+1,x) должны быть проходимы
    sdiag1 = pas[0:Hc - 1, 1:Wc]
    sdiag2 = pas[1:Hc, 0:Wc - 1]
    add((slice(0, Hc - 1), slice(0, Wc - 1)), (slice(1, Hc), slice(1, Wc)), cres * 1.4142, True)
    # диагональ (+1,-1): углы (y,x-1) и (y+1,x)
    sdiag1 = pas[0:Hc - 1, 0:Wc - 1]
    sdiag2 = pas[1:Hc, 1:Wc]
    add((slice(0, Hc - 1), slice(1, Wc)), (slice(1, Hc), slice(0, Wc - 1)), cres * 1.4142, True)

    rr = np.concatenate(rows)
    cc = np.concatenate(cols)
    ww = np.concatenate(wts)
    n = len(nodes)
    g = coo_matrix((ww, (rr, cc)), shape=(n, n)).tocsr()
    src = ids[rcy, rcx]
    dist = dijkstra(g, directed=False, indices=src)
    tcx = np.clip(np.floor((np.asarray(tx) - an.ox) / cres).astype(int), 0, Wc - 1)
    tcy = np.clip(np.floor((np.asarray(ty) - an.oy) / cres).astype(int), 0, Hc - 1)
    tid = ids[tcy, tcx]
    out = np.full(len(tid), np.inf)
    okk = tid >= 0
    out[okk] = dist[tid[okk]]
    return out

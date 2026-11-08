"""Чистые функции детектора (без ROS): пятна на тепловом кадре, дальность по лидару."""
import math

import numpy as np
from scipy import ndimage


def find_blobs(temp, hot_k, min_px):
    """temp (h, w) в K. Возвращает список пятен (связные области > hot_k, >= min_px)."""
    mask = temp > hot_k
    lab, n = ndimage.label(mask, structure=np.ones((3, 3)))
    out = []
    if n == 0:
        return out
    objs = ndimage.find_objects(lab)
    for k, sl in enumerate(objs, start=1):
        m = lab[sl] == k
        px = int(m.sum())
        if px < min_px:
            continue
        vals = temp[sl][m]
        jj, ii = np.nonzero(m)
        # ядро: верхняя половина по температуре (края размыты фоном)
        thr = vals.min() + 0.5 * (vals.max() - vals.min())
        core = vals[vals >= thr]
        out.append(dict(
            u0=sl[1].start, u1=sl[1].stop - 1, v0=sl[0].start, v1=sl[0].stop - 1,
            uc=float(ii.mean() + sl[1].start), vc=float(jj.mean() + sl[0].start),
            px=px, mean_k=float(core.mean()), max_k=float(vals.max()),
            mask=m, off=(sl[0].start, sl[1].start)))
    return out


def lidar_on_blob(pts_t, blob, fx, fy, cx, cy, cluster_m, min_z, pts_z, offset_m=0.2, gap_m=0.4):
    """pts_t (N,3) точки в кадре тепловизора (x вперёд, y влево, z вверх),
    pts_z (N,) их высота в мировой системе. Берём точки, проецирующиеся на пиксели пятна
    (сперва ровно, затем с запасом 2x4 px), выше пола, ближайший кластер по глубине.
    Возвращает dict (depth - ближняя дальность, pos - центр объекта в кадре тепловизора,
    height, length, n) или None."""
    if len(pts_t) == 0:
        return None
    x = pts_t[:, 0]
    ok = (x > 0.1) & (pts_z > min_z)
    if not ok.any():
        return None
    p, z = pts_t[ok], pts_z[ok]
    u = cx - fx * p[:, 1] / p[:, 0]
    v = cy - fy * p[:, 2] / p[:, 0]
    j0, i0 = blob['off']
    h, w = blob['mask'].shape
    sel = None
    for lvl, (du, dv) in enumerate(((0, 0), (1, 1), (2, 4))):
        pad = np.pad(blob['mask'], ((dv, dv), (du, du)))
        if du or dv:
            pad = ndimage.binary_dilation(pad, structure=np.ones((2 * dv + 1, 2 * du + 1), dtype=bool))
        jj = np.round(v).astype(int) - j0 + dv
        ii = np.round(u).astype(int) - i0 + du
        inside = (jj >= 0) & (jj < pad.shape[0]) & (ii >= 0) & (ii < pad.shape[1])
        s = np.zeros(len(p), dtype=bool)
        s[inside] = pad[jj[inside], ii[inside]]
        if s.any():
            sel = s
            break
    else:
        lvl = None
    if sel is None:
        return None
    p, z = p[sel], z[sel]
    d = np.hypot(p[:, 0], p[:, 1])
    # кластеры по глубине (разрыв > gap_m); берём самый многочисленный (притолока двери
    # или стена рядом с пятном дают малые ближние кластеры), при равенстве ближний
    order = np.argsort(d)
    ds = d[order]
    brk = np.nonzero(np.diff(ds) > gap_m)[0] + 1
    groups = np.split(order, brk)
    best = max(groups, key=lambda gi: (len(gi), -d[gi].min()))
    d0 = d[best].min()
    cl = np.zeros(len(d), dtype=bool)
    cl[best] = True
    cl &= d <= d0 + cluster_m
    p, z, d = p[cl], z[cl], d[cl]
    xy = p[:, :2]
    c = xy.mean(axis=0)
    length = 0.0
    nrm = c / max(np.hypot(*c), 1e-6)
    if len(xy) >= 5:
        cc = xy - c
        wv, vec = np.linalg.eigh(cc.T @ cc)
        proj = cc @ vec[:, 1]
        length = float(proj.max() - proj.min())
        if wv[1] > 4.0 * max(wv[0], 1e-6):   # точки лежат вдоль грани: нормаль к ней
            n2 = np.array([-vec[1, 1], vec[0, 1]])
            nrm = n2 if n2 @ c > 0 else -n2
    pos = c + offset_m * nrm
    return dict(depth=float(d0), pos=pos, height=float(z.max()), length=length, n=int(len(p)), pad=lvl)


def ground_range(blob, fx, fy, cx, cy, cam_z, max_range):
    """Запасной вариант: луч через нижнюю точку пятна до плоскости пола. cam_z - высота камеры
    над полом. Возвращает горизонтальную дальность или None. Только в кадре камеры (x вперёд)."""
    ub = (blob['u0'] + blob['u1']) / 2.0
    vb = blob['v1'] + 0.5
    dz = -(vb - cy) / fy      # вертикальная составляющая луча при x = 1
    if dz >= -1e-4:
        return None
    t = cam_z / -dz
    if t > max_range:
        return None
    return float(t * math.hypot(1.0, (cx - ub) / fx))


def lidar_consistent(depth, ground, fy, cam_z, base_m=0.3, k_px=1.2):
    """Дальность по лидару согласуется с дальностью по низу пятна (пол)? Нет - точки лидара принадлежат
    другому объекту (ближний предмет рядом с пятном на кадре, закрытое пятно), им нельзя верить.
    Допуск: base_m + k_px пикселей по вертикали (1 px = r^2 / (fy * cam_z) м на дальности r).
    ground=None (нет оценки по полу) - не проверяем."""
    if ground is None:
        return True
    r = max(depth, ground)
    tol = base_m + k_px * r * r / (fy * max(cam_z, 0.05))
    return abs(depth - ground) <= tol

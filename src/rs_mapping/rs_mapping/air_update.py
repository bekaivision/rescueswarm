"""Воздушный слой общей карты: облако nadir-глубины дрона -> log-odds.

Без ROS, чтобы можно было тестировать. Дрон смотрит вниз: из облака в мире берём
высоту над полом h, по клетке максимум h. Пол оцениваем по самому облаку (низкий
перцентиль z), поэтому ошибка высоты дрона до ~0.5 м не превращает пол в препятствия.
Пол в кадре - плоскость (fit_floor_plane): ошибка наклона дрона на несколько градусов наклоняет весь
пол в облаке, и плоский уровень на краю кадра поднимал пол выше порога препятствия.
"""
import numpy as np


def estimate_floor(z, percentile=10.0, clamp=0.6):
    """Высота пола в мире по точкам облака: низкий перцентиль z, зажатый в +-clamp."""
    if z.size == 0:
        return 0.0
    return float(np.clip(np.percentile(z, percentile), -clamp, clamp))


def fit_floor_plane(p, floor, *, band=0.35, inlier=0.06, max_tilt_deg=8.0, min_pts=200):
    """Плоскость пола z = a*x + b*y + c по точкам облака (N, 3) в мире.

    Кандидаты в пол - точки не выше floor + band (floor - низкий перцентиль z), затем два шага МНК с
    отбором точек ближе inlier к плоскости. Если точек мало или наклон больше max_tilt_deg, возвращаем
    горизонтальную плоскость на уровне floor. Возвращает (a, b, c).
    """
    flat = (0.0, 0.0, float(floor))
    sel = p[p[:, 2] <= floor + band]
    if len(sel) < min_pts:
        return flat
    x0, y0 = float(sel[:, 0].mean()), float(sel[:, 1].mean())
    coef = None
    for _ in range(2):
        A = np.column_stack([sel[:, 0] - x0, sel[:, 1] - y0, np.ones(len(sel))])
        coef, *_ = np.linalg.lstsq(A, sel[:, 2], rcond=None)
        res = sel[:, 2] - A @ coef
        nxt = sel[np.abs(res) <= inlier]
        if len(nxt) < min_pts:
            break
        sel = nxt
    a, b, c0 = (float(v) for v in coef)
    if np.degrees(np.arctan(np.hypot(a, b))) > max_tilt_deg:
        return flat
    return a, b, c0 - a * x0 - b * y0


class AirLayer:
    """Слой log-odds по дронам (те же клетки и константы, что у наземного слоя)."""

    def __init__(self, width, height, res, origin_x, origin_y, *, hit=0.85, miss=0.3,
                 l_min=-2.0, l_max=3.5, occ_thr=0.4, obst_h=0.15, free_h=0.08, max_h=3.0,
                 ground_clear=0.4, drone_clear=0.6, floor_percentile=10.0, floor_clamp=0.6,
                 max_off_nadir_deg=35.0, floor_plane=True):
        self.w, self.h, self.res = width, height, res
        self.ox, self.oy = origin_x, origin_y
        self.hit, self.miss = hit, miss
        self.l_min, self.l_max, self.occ_thr = l_min, l_max, occ_thr
        self.obst_h, self.free_h, self.max_h = obst_h, free_h, max_h
        self.ground_clear, self.drone_clear = ground_clear, drone_clear
        self.floor_percentile, self.floor_clamp = floor_percentile, floor_clamp
        self.max_off_nadir = np.radians(max_off_nadir_deg)
        self.floor_plane = floor_plane
        self.logodds = np.zeros(width * height, dtype=np.float32)
        self.seen = np.zeros(width * height, dtype=bool)

    def to_grid_data(self):
        """int8: -1 неизвестно, 0 свободно, 100 занято."""
        d = np.full(self.w * self.h, -1, dtype=np.int8)
        d[self.seen] = 0
        d[self.seen & (self.logodds >= self.occ_thr)] = 100
        return d

    def update(self, world_pts, ground_xy=(), drone_xy=(), sensor_xyz=None):
        """world_pts: (N, 3) точки в кадре world (NaN допустимы).

        ground_xy: наземные роботы (сверху похожи на препятствия, радиус ground_clear);
        drone_xy: другие дроны (радиус drone_clear); sensor_xyz: положение сенсора в мире. Если задано и
        max_off_nadir > 0, берём только точки в конусе max_off_nadir от вертикали: на краю широкого кадра
        ошибка наклона дрона в 2-3 градуса поднимает пол на 0.15-0.2 м (ложные препятствия).
        Возвращает (занято, свободно) клеток.
        """
        p = world_pts.reshape(-1, 3)
        ok = np.isfinite(p).all(axis=1)
        for (rx, ry) in ground_xy:
            ok &= np.hypot(p[:, 0] - rx, p[:, 1] - ry) >= self.ground_clear
        for (rx, ry) in drone_xy:
            ok &= np.hypot(p[:, 0] - rx, p[:, 1] - ry) >= self.drone_clear
        p = p[ok]
        if len(p) == 0:
            return 0, 0
        floor = estimate_floor(p[:, 2], self.floor_percentile, self.floor_clamp)
        if sensor_xyz is not None and self.max_off_nadir > 0:
            sx, sy, sz = sensor_xyz
            r_max = max(sz - floor, 0.0) * np.tan(self.max_off_nadir)
            p = p[np.hypot(p[:, 0] - sx, p[:, 1] - sy) <= r_max]
            if len(p) == 0:
                return 0, 0
        if self.floor_plane:
            a, b, c = fit_floor_plane(p, floor)
            hh = p[:, 2] - (a * p[:, 0] + b * p[:, 1] + c)
        else:
            hh = p[:, 2] - floor
        keep = hh <= self.max_h
        p, hh = p[keep], hh[keep]
        ix = np.floor((p[:, 0] - self.ox) / self.res).astype(np.int64)
        iy = np.floor((p[:, 1] - self.oy) / self.res).astype(np.int64)
        inside = (ix >= 0) & (ix < self.w) & (iy >= 0) & (iy < self.h)
        if not inside.any():
            return 0, 0
        idx = iy[inside] * self.w + ix[inside]
        hh = hh[inside]
        cells, inv = np.unique(idx, return_inverse=True)
        hmax = np.full(len(cells), -np.inf)
        np.maximum.at(hmax, inv, hh)
        occ = cells[hmax >= self.obst_h]
        free = cells[hmax <= self.free_h]
        L = self.logodds
        L[free] = np.maximum(L[free] - self.miss, self.l_min)
        L[occ] = np.minimum(L[occ] + self.hit, self.l_max)
        self.seen[free] = True
        self.seen[occ] = True
        return len(occ), len(free)


def fuse(ground, air):
    """Слияние слоёв: клетка известна на земле -> значение земли, иначе значение воздуха."""
    g = ground.to_grid_data()
    a = air.to_grid_data()
    return np.where(ground.seen, g, a).astype(np.int8)

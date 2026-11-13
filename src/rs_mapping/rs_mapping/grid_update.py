"""Чистая численная часть общей карты: один скан лидара -> обновление log-odds.

Без ROS, чтобы можно было тестировать. Сетка хранится плоским массивом
(индекс = iy * width + ix), мировая клетка (ix, iy) = floor((p - origin) / res).
"""
import numpy as np


def cloud_to_array(data, width, height, point_step, offsets):
    """PointCloud2 -> массив (height, width, 3) float32 (x, y, z); нет возврата = NaN."""
    buf = np.frombuffer(data, dtype=np.uint8)
    n = width * height
    raw = buf[:n * point_step].reshape(n, point_step)
    out = np.empty((n, 3), dtype=np.float32)
    for k, off in enumerate(offsets):
        out[:, k] = raw[:, off:off + 4].copy().view(np.float32)[:, 0]
    out[~np.isfinite(out).all(axis=1)] = np.nan
    return out.reshape(height, width, 3)


class GridUpdater:
    def __init__(self, width, height, res, origin_x, origin_y, *, z_min=0.05, z_max=1.5,
                 hit=0.85, miss=0.3, l_min=-2.0, l_max=3.5, occ_thr=0.4,
                 subrays=5, max_range=30.0, robot_clear=0.4, self_clear=0.3, min_range=0.12,
                 free_z_max=2.5, drone_clear_xy=0.5, drone_clear_z=0.5):
        self.w, self.h, self.res = width, height, res
        self.ox, self.oy = origin_x, origin_y
        self.z_min, self.z_max = z_min, z_max
        self.hit, self.miss = hit, miss
        self.l_min, self.l_max, self.occ_thr = l_min, l_max, occ_thr
        self.subrays, self.max_range = subrays, max_range
        self.robot_clear, self.self_clear, self.min_range = robot_clear, self_clear, min_range
        # выше free_z_max точки не годятся для свободного пространства: это летящий дрон,
        # а не стена (в мирах без дронов выше 2.0 м ничего нет, поведение то же)
        self.free_z_max = free_z_max
        self.drone_clear_xy, self.drone_clear_z = drone_clear_xy, drone_clear_z
        self.logodds = np.zeros(width * height, dtype=np.float32)
        self.seen = np.zeros(width * height, dtype=bool)
        self._tmp = np.zeros(width * height, dtype=bool)

    # --- публикуемая карта -------------------------------------------------
    def to_grid_data(self):
        """int8: -1 неизвестно, 0 свободно, 100 занято."""
        d = np.full(self.w * self.h, -1, dtype=np.int8)
        d[self.seen] = 0
        d[self.seen & (self.logodds >= self.occ_thr)] = 100
        return d

    # --- обновление --------------------------------------------------------
    def _cells(self, x, y):
        ix = np.floor((x - self.ox) / self.res).astype(np.int64)
        iy = np.floor((y - self.oy) / self.res).astype(np.int64)
        ok = (ix >= 0) & (ix < self.w) & (iy >= 0) & (iy < self.h)
        return ix, iy, ok

    def update(self, pts, T, other_robots=(), self_xy=None, drones=()):
        """pts: (rows, cols, 3) в кадре лидара (NaN = нет возврата), T: 4x4 мир <- лидар.

        other_robots: список (x, y) других роботов в мире: точки рядом с корпусом отбрасываем
        (радиус robot_clear); self_xy: свой робот (радиус self_clear, меньше: робот может ехать у стены).
        drones: список (x, y, z) дронов в мире: точки в 0.5 м по xy и 0.5 м по z отбрасываем
        (дрон на земле до взлёта не должен стать вечным препятствием).
        Возвращает (число занятых клеток, число свободных клеток) этого скана.
        """
        rows, cols, _ = pts.shape
        R, t = T[:3, :3], T[:3, 3]
        flat = pts.reshape(-1, 3)
        fin = np.isfinite(flat[:, 0])
        world = np.full_like(flat, np.nan)
        world[fin] = flat[fin] @ R.T + t
        rng = np.hypot(flat[:, 0], flat[:, 1])
        fin &= (rng > self.min_range) & (rng < self.max_range)

        # точки на корпусах роботов (чужие и свои) не считаем ни препятствиями, ни дальностью
        for (rx, ry) in other_robots:
            near = np.hypot(world[:, 0] - rx, world[:, 1] - ry) < self.robot_clear
            fin &= ~near
        if self_xy is not None:
            near = np.hypot(world[:, 0] - self_xy[0], world[:, 1] - self_xy[1]) < self.self_clear
            fin &= ~near

        for (dx_, dy_, dz_) in drones:
            near = ((np.hypot(world[:, 0] - dx_, world[:, 1] - dy_) < self.drone_clear_xy)
                    & (np.abs(world[:, 2] - dz_) < self.drone_clear_z))
            fin &= ~near
        # слишком высокие точки (летящий дрон) не режут свободное пространство
        fin &= world[:, 2] <= self.free_z_max

        obst = fin & (world[:, 2] >= self.z_min) & (world[:, 2] <= self.z_max)
        col = np.tile(np.arange(cols), rows)

        # по каждому столбцу (азимуту): ближайшее препятствие и дальняя точка
        inf = np.inf
        obst_min = np.full(cols, inf)
        np.minimum.at(obst_min, col[obst], rng[obst])
        far = np.full(cols, -1.0)
        np.maximum.at(far, col[fin], rng[fin])
        free_r = np.where(np.isfinite(obst_min), obst_min - self.res, far)
        free_r = np.clip(free_r, 0.0, self.max_range)
        free_r[far < 0] = 0.0

        # азимут каждого столбца: среднее направление его точек
        ux = np.zeros(cols)
        uy = np.zeros(cols)
        np.add.at(ux, col[fin], flat[fin, 0] / rng[fin])
        np.add.at(uy, col[fin], flat[fin, 1] / rng[fin])
        have = (ux != 0) | (uy != 0)
        if not have.any():
            return 0, 0
        dth = 2 * np.pi / cols
        meas = np.arctan2(uy, ux)
        off = np.angle(np.mean(np.exp(1j * (meas[have] - np.arange(cols)[have] * dth))))
        az = np.arange(cols) * dth + off

        # подлучи: между столбцами c и c+1 берём минимум дальностей (консервативно)
        S = self.subrays
        nxt = np.roll(free_r, -1)
        r_sub = np.empty((cols, S))
        r_sub[:, 0] = free_r
        r_sub[:, 1:] = np.minimum(free_r, nxt)[:, None]
        a_sub = az[:, None] + (np.arange(S)[None, :] / S) * dth
        r_sub = r_sub.ravel()
        a_sub = a_sub.ravel()
        keep = r_sub > self.res
        r_sub, a_sub = r_sub[keep], a_sub[keep]

        # направление луча из кадра лидара в мир
        dx_l, dy_l = np.cos(a_sub), np.sin(a_sub)
        dx = R[0, 0] * dx_l + R[0, 1] * dy_l
        dy = R[1, 0] * dx_l + R[1, 1] * dy_l
        norm = np.hypot(dx, dy)
        dx, dy = dx / norm, dy / norm
        rw = r_sub * norm  # дальность по горизонтали в мире (при крене лидара чуть меньше)

        # клетки вдоль лучей: шаг = res/2 по длине, у каждого луча своё число шагов
        step = self.res * 0.5
        cnt = np.maximum((rw / step).astype(np.int64), 1)
        total = int(cnt.sum())
        ridx = np.repeat(np.arange(len(rw)), cnt)
        starts = np.cumsum(cnt) - cnt
        k = np.arange(total) - np.repeat(starts, cnt)
        s = k * step
        px = t[0] + dx[ridx] * s
        py = t[1] + dy[ridx] * s
        ix, iy, ok = self._cells(px, py)
        free = np.unique(iy[ok] * self.w + ix[ok])

        # занятые клетки: все точки-препятствия
        ox_, oy_, ook = self._cells(world[obst, 0], world[obst, 1])
        occ = np.unique(oy_[ook] * self.w + ox_[ook])

        # занято сильнее свободного в одном скане
        self._tmp[occ] = True
        free = free[~self._tmp[free]]
        self._tmp[occ] = False

        L = self.logodds
        L[free] = np.maximum(L[free] - self.miss, self.l_min)
        L[occ] = np.minimum(L[occ] + self.hit, self.l_max)
        self.seen[free] = True
        self.seen[occ] = True
        return len(occ), len(free)

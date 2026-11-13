import numpy as np

from rs_mapping.grid_update import GridUpdater


def _scan(cols=360, rows=4, wall_x=3.0):
    """Лидар в начале координат, стена x = wall_x, высота точек 0.5 м."""
    pts = np.full((rows, cols, 3), np.nan, dtype=np.float32)
    for c in range(cols):
        a = -np.pi + (c + 0.5) * 2 * np.pi / cols
        if np.cos(a) > 0.05:
            r = wall_x / np.cos(a)
            if r < 30:
                for k in range(rows):
                    pts[k, c] = (r * np.cos(a), r * np.sin(a), 0.5)
    return pts


def test_wall_occupied_and_free_before_it():
    up = GridUpdater(200, 200, 0.05, -5.0, -5.0)
    T = np.eye(4)
    T[2, 3] = 0.5
    for _ in range(3):
        up.update(_scan(), T)
    d = up.to_grid_data().reshape(200, 200)
    iy0 = int((0.0 + 5) / 0.05)
    ix_wall = int((3.0 + 5) / 0.05)
    assert d[iy0, ix_wall] == 100 or d[iy0, ix_wall - 1] == 100
    assert d[iy0, int((1.5 + 5) / 0.05)] == 0
    assert d[iy0, int((-2.0 + 5) / 0.05)] == -1      # позади робота ничего не видно


def test_other_robot_points_dropped():
    up = GridUpdater(200, 200, 0.05, -5.0, -5.0)
    T = np.eye(4)
    T[2, 3] = 0.5
    up.update(_scan(wall_x=1.0), T, other_robots=[(1.0, 0.0)])
    d = up.to_grid_data().reshape(200, 200)
    assert d[100, int((1.0 + 5) / 0.05)] != 100


def test_high_returns_do_not_carve_free_space():
    # попадание в летящий дрон (z = 3.5 м) далеко за стеной не должно резать свободное пространство
    up = GridUpdater(200, 200, 0.05, -5.0, -5.0)
    T = np.eye(4)
    T[2, 3] = 0.5
    s = _scan(wall_x=1.0)
    s[:, :, 2] = 0.5
    # столбцы справа от оси: добавим высокие точки на 4 м (нет препятствий в диапазоне z)
    hi = np.full((1, 360, 3), np.nan, dtype=np.float32)
    for c in range(360):
        a = -np.pi + (c + 0.5) * 2 * np.pi / 360
        if abs(a - np.pi / 2) < 0.05:
            hi[0, c] = (4.0 * np.cos(a), 4.0 * np.sin(a), 3.5)
    up.update(np.concatenate([s, hi], axis=0), T)
    d = up.to_grid_data().reshape(200, 200)
    assert d[int((3.0 + 5) / 0.05), 100] == -1       # вдоль +y высокая точка свободу не дала


def test_free_z_max_does_not_change_scan_below_it():
    # все точки ниже 2 м: результат с free_z_max=2.5 и без ограничения совпадает бит в бит
    a = GridUpdater(200, 200, 0.05, -5.0, -5.0)
    b = GridUpdater(200, 200, 0.05, -5.0, -5.0, free_z_max=1e9)
    T = np.eye(4)
    T[2, 3] = 0.5
    s = _scan()
    s[0, :, 2] = 1.9
    for up in (a, b):
        up.update(s, T, other_robots=[(1.0, 1.0)], self_xy=(0.0, 0.0))
    assert np.array_equal(a.logodds, b.logodds) and np.array_equal(a.seen, b.seen)


def test_parked_drone_points_dropped():
    up = GridUpdater(200, 200, 0.05, -5.0, -5.0)
    T = np.eye(4)
    T[2, 3] = 0.5
    up.update(_scan(wall_x=1.0), T, drones=[(1.0, 0.0, 1.0)])
    d = up.to_grid_data().reshape(200, 200)
    assert d[100, int((1.0 + 5) / 0.05)] != 100
    # дрон высоко над точкой: точка остаётся препятствием
    up2 = GridUpdater(200, 200, 0.05, -5.0, -5.0)
    up2.update(_scan(wall_x=1.0), T, drones=[(1.0, 0.0, 4.5)])
    d2 = up2.to_grid_data().reshape(200, 200)
    assert d2[100, int((1.0 + 5) / 0.05)] == 100 or d2[100, int((1.0 + 5) / 0.05) - 1] == 100

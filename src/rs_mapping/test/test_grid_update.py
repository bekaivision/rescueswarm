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

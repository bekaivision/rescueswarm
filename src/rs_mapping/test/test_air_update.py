import numpy as np
import pytest

from rs_mapping.air_update import AirLayer, estimate_floor, fuse
from rs_mapping.grid_update import GridUpdater


def _layer():
    return AirLayer(200, 200, 0.05, -5.0, -5.0)


def _cloud(bias=0.0, wall=True):
    """Вид сверху: пол z = bias (на сетке 4x4 м), у x in [1, 1.2] стена высотой 2 м (верх)."""
    xs, ys = np.meshgrid(np.arange(-2.0, 2.0, 0.05), np.arange(-2.0, 2.0, 0.05))
    x, y = xs.ravel(), ys.ravel()
    z = np.full_like(x, bias)
    if wall:
        w = (x >= 1.0) & (x < 1.2)
        z[w] = bias + 2.0
    return np.stack([x, y, z], axis=1)


def _cell(layer, x, y):
    return int((y + 5) / 0.05) * layer.w + int((x + 5) / 0.05)


def test_floor_estimate_follows_bias_and_is_clamped():
    z = np.random.default_rng(0).normal(0.0, 0.01, 1000)
    assert estimate_floor(z + 0.5) == pytest.approx(0.5, abs=0.05)
    assert estimate_floor(z - 0.5) == pytest.approx(-0.5, abs=0.05)
    assert estimate_floor(z + 3.0) == 0.6
    assert estimate_floor(np.array([])) == 0.0


@pytest.mark.parametrize('bias', [-0.5, -0.2, 0.0, 0.3, 0.5])
def test_wall_top_occupied_floor_free_under_altitude_bias(bias):
    a = _layer()
    for _ in range(2):
        a.update(_cloud(bias))
    d = a.to_grid_data()
    assert d[_cell(a, 1.1, 0.0)] == 100            # верх стены
    assert d[_cell(a, 0.0, 0.0)] == 0              # пол
    assert d[_cell(a, 3.0, 3.0)] == -1             # вне кадра неизвестно
    assert (d[a.seen] == 100).sum() == (d == 100).sum()
    n_occ = int(((d == 100)).sum())
    assert n_occ < 0.1 * a.seen.sum()              # пол не превратился в препятствия


def test_hysteresis_band_not_updated():
    a = _layer()
    p = _cloud(wall=False)
    p[:, 2] += 0.0
    # одна клетка на высоте 0.1 м над полом (между free_h 0.08 и obst_h 0.15)
    p[(np.abs(p[:, 0] - 0.0) < 0.02) & (np.abs(p[:, 1] - 0.0) < 0.02), 2] = 0.1
    a.update(p)
    assert not a.seen[_cell(a, 0.0, 0.0)]


def test_points_above_max_h_ignored():
    a = _layer()
    p = _cloud(wall=False)
    p = np.vstack([p, [[0.5, 0.5, 5.0]]])           # высоко над полом (птица, провод)
    a.update(p)
    assert a.to_grid_data()[_cell(a, 0.5, 0.5)] == 0


def test_masking_ground_robot_and_other_drone():
    a = _layer()
    p = _cloud(wall=False)
    # "корпус" робота: точки 0.3 м над полом в (0, 0)
    body = np.array([[0.0, 0.0, 0.3], [0.1, 0.0, 0.3]])
    a.update(np.vstack([p, body]), ground_xy=[(0.0, 0.0)])
    assert a.to_grid_data()[_cell(a, 0.0, 0.0)] != 100
    b = _layer()
    b.update(np.vstack([p, body]))
    assert b.to_grid_data()[_cell(b, 0.0, 0.0)] == 100
    # другой дрон: радиус 0.6 м
    c = _layer()
    far = np.array([[0.5, 0.0, 1.0]])
    c.update(np.vstack([p, far]), drone_xy=[(0.0, 0.0)])
    assert c.to_grid_data()[_cell(c, 0.5, 0.0)] != 100
    d = _layer()
    d.update(np.vstack([p, far]), ground_xy=[(0.0, 0.0)])   # для наземного радиус только 0.4
    assert d.to_grid_data()[_cell(d, 0.5, 0.0)] == 100


def test_nan_points_and_empty():
    a = _layer()
    assert a.update(np.full((10, 3), np.nan)) == (0, 0)
    assert a.update(np.zeros((0, 3))) == (0, 0)


def test_fusion_rule():
    g = GridUpdater(10, 10, 0.05, 0.0, 0.0)
    a = AirLayer(10, 10, 0.05, 0.0, 0.0)
    # клетка 0: земля занято, воздух свободно -> 100; клетка 1: земля свободно, воздух занято -> 0
    # клетка 2: земля неизвестно, воздух занято -> 100; клетка 3: земля неизвестно, воздух свободно -> 0
    # клетка 4: оба неизвестно -> -1
    g.seen[[0, 1]] = True
    g.logodds[0], g.logodds[1] = 2.0, -1.0
    a.seen[[0, 1, 2, 3]] = True
    a.logodds[[0, 1, 2, 3]] = [-1.0, 2.0, 2.0, -1.0]
    f = fuse(g, a)
    assert list(f[:5]) == [100, 0, 100, 0, -1]
    assert f.dtype == np.int8


def test_empty_air_robots_means_ground_layer_unchanged():
    # без воздушного слоя /map = to_grid_data() наземного слоя (shared_map не вызывает fuse)
    g = GridUpdater(10, 10, 0.05, 0.0, 0.0)
    g.seen[:3] = True
    g.logodds[0] = 2.0
    empty_air = AirLayer(10, 10, 0.05, 0.0, 0.0)
    assert np.array_equal(fuse(g, empty_air), g.to_grid_data())


def test_off_nadir_cone_drops_frame_edges():
    # дрон на 4.5 м над полом; пол поднят на краю кадра (ошибка наклона), а под дроном ровный
    from rs_mapping.air_update import AirLayer
    lay = AirLayer(200, 200, 0.05, -5.0, -5.0, max_off_nadir_deg=35.0)
    xs = np.linspace(-4.0, 4.0, 161)
    pts = np.array([(x, 0.0, 0.25 if abs(x) > 3.5 else 0.0) for x in xs])   # края кадра "подняты" на 0.25 м
    occ, free = lay.update(pts, sensor_xyz=(0.0, 0.0, 4.5))
    assert occ == 0 and free > 0                                    # конус 35 град на 4.5 м: |x| <= 3.15 м, пол ровный
    lay2 = AirLayer(200, 200, 0.05, -5.0, -5.0, max_off_nadir_deg=0.0)
    occ2, _ = lay2.update(pts, sensor_xyz=(0.0, 0.0, 4.5))
    assert occ2 > 0                                                 # без конуса края дают ложные препятствия


def _tilted(deg, box=True):
    """Пол наклонён на deg градусов по x (ошибка наклона дрона), ящик 0.5x0.5x0.4 м в (-1, -1)."""
    xs, ys = np.meshgrid(np.arange(-3.0, 3.0, 0.05), np.arange(-3.0, 3.0, 0.05))
    x, y = xs.ravel(), ys.ravel()
    z = np.tan(np.radians(deg)) * x
    if box:
        b = (np.abs(x + 1.0) < 0.25) & (np.abs(y + 1.0) < 0.25)
        z[b] += 0.4
    return np.stack([x, y, z], axis=1)


@pytest.mark.parametrize('deg', [0.0, 2.0, 4.0, -4.0])
def test_floor_plane_removes_tilt(deg):
    from rs_mapping.air_update import fit_floor_plane
    a, b, c = fit_floor_plane(_tilted(deg), estimate_floor(_tilted(deg)[:, 2]))
    assert np.degrees(np.arctan(a)) == pytest.approx(deg, abs=0.2)
    assert b == pytest.approx(0.0, abs=0.01)
    lay = AirLayer(200, 200, 0.05, -5.0, -5.0)
    lay.update(_tilted(deg))
    d = lay.to_grid_data()
    assert d[_cell(lay, -1.0, -1.0)] == 100          # ящик виден
    assert d[_cell(lay, 2.8, 2.0)] == 0              # край кадра - пол, не препятствие
    assert (d == 100).sum() < 200                    # занят только ящик (~100 клеток)


def test_flat_floor_level_under_tilt_gives_false_obstacles():
    lay = AirLayer(200, 200, 0.05, -5.0, -5.0, floor_plane=False)
    lay.update(_tilted(4.0))
    assert lay.to_grid_data()[_cell(lay, 2.8, 2.0)] == 100   # без плоскости поднятый край кадра - "препятствие"


def test_floor_plane_falls_back_when_too_steep_or_few_points():
    from rs_mapping.air_update import fit_floor_plane
    assert fit_floor_plane(_tilted(15.0, box=False), 0.0) == (0.0, 0.0, 0.0)
    assert fit_floor_plane(np.zeros((10, 3)), 0.2) == (0.0, 0.0, 0.2)

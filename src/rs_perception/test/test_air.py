import math

import numpy as np
import pytest

from rs_perception import marks as M
from rs_perception.air_geometry import pixel_floor_hit, touches_border

K = (100.0, 100.0, 79.5, 59.5)


def _nadir(x=0.0, y=0.0, z=4.5, yaw=0.0):
    """Камера дрона: ось x вниз. Поворот вокруг y на +90 градусов (x -> -z)."""
    Ry = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=float)   # ось x камеры -> -z мира
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    T = np.eye(4)
    T[:3, :3] = Rz @ Ry
    T[:3, 3] = (x, y, z)
    return T


def test_exact_nadir_center_pixel():
    r = pixel_floor_hit(79.5, 59.5, K, _nadir(1.0, 2.0, 4.5))
    assert r is not None
    p, rng = r
    assert p[:2] == pytest.approx([1.0, 2.0], abs=1e-9) and p[2] == pytest.approx(0.0)
    assert rng == pytest.approx(4.5)


def test_off_centre_pixel_nadir():
    # пиксель правее центра на 50 px: y камеры = -0.5 (вправо), при yaw=0 это -y мира? проверим по формуле
    T = _nadir(0.0, 0.0, 4.0)
    p, rng = pixel_floor_hit(79.5 + 50.0, 59.5, K, T)
    d_cam = np.array([1.0, -0.5, 0.0])
    expect = (T[:3, :3] @ d_cam) * 4.0 / 1.0
    assert p[:2] == pytest.approx(expect[:2], abs=1e-9)
    assert math.hypot(*p[:2]) == pytest.approx(2.0)
    assert rng == pytest.approx(4.0 * math.sqrt(1.25))


def test_tilt_10_degrees():
    T = _nadir(0.0, 0.0, 4.0)
    a = math.radians(10)
    # наклон вокруг оси y мира на 10 градусов: центральный луч уходит в сторону на 4*tan(10)
    Rt = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
    T2 = T.copy()
    T2[:3, :3] = Rt @ T[:3, :3]
    p, rng = pixel_floor_hit(79.5, 59.5, K, T2)
    assert abs(p[0]) == pytest.approx(4.0 * math.tan(a), abs=1e-6) and abs(p[1]) < 1e-9
    assert rng == pytest.approx(4.0 / math.cos(a))


def test_ray_misses_floor():
    # горизонтальная камера: луч не идёт вниз
    T = np.eye(4)
    T[2, 3] = 2.0
    assert pixel_floor_hit(79.5, 59.5, K, T) is None
    # луч вверх (пиксель выше центра при горизонтальной оси)
    assert pixel_floor_hit(79.5, 0.0, K, T) is None
    # камера ниже пола, смотрит вниз: пересечение позади
    assert pixel_floor_hit(79.5, 59.5, K, _nadir(z=-1.0)) is None


def test_touches_border():
    w, h = 160, 120
    b = dict(u0=10, u1=20, v0=10, v1=20)
    assert not touches_border(b, w, h)
    assert touches_border(dict(b, u0=0), w, h)
    assert touches_border(dict(b, v1=119), w, h)
    assert touches_border(dict(b, u1=159), w, h)


# --- правила воздуха в marks ---

def fa(**kw):
    return M.Fusion(air_robots=['drone_1'], **kw)


def air(f, n, x=1.0, y=2.0, rng=4.5, temp=310.0, t0=0.0):
    nan = float('nan')
    for i in range(n):
        m, _ = f.update(x, y, 'drone_1', rng, temp, nan, nan, t0 + i)
    return m


def ground(f, n, x=1.0, y=2.0, rng=2.0, temp=310.0, h=0.2, ln=1.5, t0=100.0):
    for i in range(n):
        m, _ = f.update(x, y, 'ground_1', rng, temp, h, ln, t0 + i)
    return m


def test_air_creates_candidate_but_never_confirms():
    f = fa()
    m = air(f, 30, rng=1.0)             # даже очень близкая дальность: не близкий замер
    assert m.status == M.CANDIDATE and m.n_close == 0 and m.heights == [] and m.lengths == []
    assert f.needs_inspection() == [m] and m.n_air == 30


def test_air_height_samples_not_recorded():
    f = fa()
    m, _ = f.update(1.0, 2.0, 'drone_1', 2.0, 310.0, 0.2, 1.5, 0.0)
    assert m.heights == [] and m.lengths == [] and m.n_close == 0


def test_air_too_hot_rejects():
    f = fa()
    m = air(f, 3, temp=335.0)
    assert m.status == M.REJECTED and m.reason == 'too_hot'


def test_air_does_not_reset_or_make_negative_evidence():
    f = fa()
    air(f, 2)
    m = f.marks[0]
    m.neg = {'ground_1': (1.0, 5.0)}
    air(f, 1, t0=6.0)
    assert m.neg == {'ground_1': (1.0, 5.0)}      # воздух не сбрасывает наземное накопление


def test_air_then_ground_one_mark_confirmed():
    f = fa()
    air(f, 5, x=0.0, y=0.0, temp=310.0)              # центр тела
    m = ground(f, 3, x=0.85, y=0.0, temp=311.5)       # ближний край, до 0.9 м от центра
    assert len(f.marks) == 1 and m.status == M.CONFIRMED


def test_ground_then_air_one_mark():
    f = fa()
    ground(f, 3, x=0.9, y=0.0)
    air(f, 5, x=0.0, y=0.0)
    assert len(f.marks) == 1 and f.marks[0].status == M.CONFIRMED


def test_air_then_ground_far_end_merges_even_with_temp_difference():
    f = fa()
    air(f, 5, x=0.0, y=0.0, temp=309.0)
    ground(f, 3, x=0.9, y=0.0, temp=312.0)
    ground(f, 3, x=-0.9, y=0.0, temp=312.0)           # с другой стороны тела
    assert len(f.marks) == 1


def test_ground_ends_wide_with_air_centre_merge():
    # две наземные отметки по концам, воздух по центру между ними
    f = fa()
    ground(f, 3, x=-0.9, y=0.0)
    ground(f, 3, x=0.9, y=0.0)
    air(f, 3, x=0.0, y=0.0)
    assert len(f.marks) == 1


def test_empty_air_robots_identical_to_before():
    # без air_robots дрон-имя ведёт себя как обычный робот
    f, g = M.Fusion(), M.Fusion(air_robots=())
    for fu in (f, g):
        for i in range(3):
            fu.update(1.0, 2.0, 'drone_1', 2.0, 310.0, 0.2, 1.5, float(i))
    assert f.marks[0].status == g.marks[0].status == M.CONFIRMED
    assert f.marks[0].n_air == 0 and f.marks[0].n_close == 3

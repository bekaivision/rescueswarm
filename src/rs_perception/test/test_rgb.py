import math

from rs_perception import marks as M

NAN = float('nan')


def thermal(f, n, x=2.0, y=0.0, rng=2.0, temp=310.0, h=NAN, ln=NAN, t0=0.0):
    for i in range(n):
        m, _ = f.update(x, y, 'ground_1', rng, temp, h, ln, t0 + i)
    return m


def rgb(f, n=1, bearing=0.0, half_w=0.1, rx=0.0, ry=0.0, yaw=0.0, t0=10.0):
    out = None
    for i in range(n):
        out = f.rgb_observe('ground_1', rx, ry, yaw, bearing, half_w, 0.6, t0 + i)
    return out


def test_rgb_confirms_when_height_unknown():
    f = M.Fusion()
    m = thermal(f, 4)
    assert m.status == M.CANDIDATE and m.n_close >= 3
    assert rgb(f, 2) is m and m.status == M.CANDIDATE
    rgb(f, 1)
    assert m.status == M.CONFIRMED and m.reason == 'temp_and_rgb' and m.rgb_hits == 3
    assert f.confidence(m) >= 0.9


def test_rgb_needs_close_obs_and_never_creates_marks():
    f = M.Fusion()
    m = thermal(f, 3, rng=6.0, x=4.0)       # дальние виды: n_close = 0
    rgb(f, 5, ry=0.0)
    assert m.status == M.CANDIDATE
    assert rgb(M.Fusion(), 3) is None and f.marks == [m]
    g = M.Fusion()
    rgb(g, 3)
    assert g.marks == [] and g.rgb_unmatched == 3


def test_rgb_does_not_override_rejections():
    for kw in (dict(temp=335.0), dict(temp=300.0), dict(temp=309.0, h=0.5, ln=0.7)):
        f = M.Fusion()
        m = thermal(f, 4, **kw)
        assert m.status == M.REJECTED
        assert rgb(f, 5) is None            # отвергнутые не привязываются
        assert m.status == M.REJECTED and m.rgb_hits == 0


def test_rgb_resets_negative_evidence():
    f = M.Fusion()
    m = thermal(f, 2, rng=6.0)
    m.neg = {'ground_1': (1.5, 5.0)}
    rgb(f, 1)
    assert m.neg == {}


def test_rgb_angle_gate_and_nearest():
    f = M.Fusion()
    a = thermal(f, 2, x=2.0, y=0.0)
    thermal(f, 2, x=4.0, y=0.0)      # дальше по тому же лучу
    assert len(f.marks) == 2
    assert f.rgb_observe('ground_1', 0.0, 0.0, 0.0, 0.0, 0.05, 0.5, 1.0) is a    # ближняя
    assert f.rgb_observe('ground_1', 0.0, 0.0, 0.0, 0.5, 0.05, 0.5, 1.0) is None  # мимо по углу
    assert f.rgb_unmatched == 1
    # слева (+y) положительный угол
    b = thermal(f, 2, x=2.0, y=2.5)
    ang = math.atan2(2.5, 2.0 - 0.25)
    assert f.rgb_observe('ground_1', 0.0, 0.0, 0.0, ang, 0.02, 0.5, 2.0) is b


def test_rgb_los_blocks():
    f = M.Fusion()
    thermal(f, 2)
    assert f.rgb_observe('ground_1', 0.0, 0.0, 0.0, 0.0, 0.1, 0.5, 1.0, los=lambda *a: False) is None


def test_rgb_merge_sums():
    f = M.Fusion()
    a = thermal(f, 3, x=0.0, h=0.2, ln=1.5)
    b = thermal(f, 3, x=2.2, h=0.2, ln=1.5)
    assert len(f.marks) == 2
    a.rgb_hits, a.rgb_score_max, a.rgb_robots = 2, 0.4, ['ground_1']
    b.rgb_hits, b.rgb_score_max, b.rgb_robots = 3, 0.7, ['ground_2']
    thermal(f, 3, x=1.1, h=0.2, ln=1.5)
    assert len(f.marks) == 1
    m = f.marks[0]
    assert m.rgb_hits == 5 and m.rgb_score_max == 0.7 and set(m.rgb_robots) == {'ground_1', 'ground_2'}


def test_confidence_bonus_candidate():
    f = M.Fusion()
    m = thermal(f, 2, rng=6.0)
    c0 = f.confidence(m)
    m.rgb_hits = 3
    assert f.confidence(m) > c0
    m.status = M.REJECTED
    assert f.confidence(m) == 0.05

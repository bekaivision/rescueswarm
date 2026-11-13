"""Фаза 7a: второе тело на одной отметке (A), пересмотр CONFIRMED по высоте (B), слияние дальних отметок (C)."""
import math

from rs_perception import marks as M

NAN = float('nan')


def feed(f, n, x=0.0, y=0.0, rng=2.0, temp=310.97, h=0.2, ln=1.5, t0=0.0, robot='r1'):
    m = None
    for i in range(n):
        m, _ = f.update(x, y, robot, rng, temp, h, ln, t0 + i * 0.2)
    return m


# --- A: отвергнутая по высоте/жару отметка вплотную к CONFIRMED - тот же человек ---

def test_a_rejected_second_mark_of_same_body_is_absorbed():
    f = M.Fusion()
    a = feed(f, 3, x=0.0)
    assert a.status == M.CONFIRMED
    # вторая отметка на том же теле в 1.2 м (дальше gate_m): ранние выбросы высоты отвергают её
    b = feed(f, 3, x=1.2, h=0.6, t0=10.0)
    assert len(f.marks) == 1 and f.marks[0].status == M.CONFIRMED
    assert b is f.marks[0]


def test_a_far_rejected_neighbour_is_not_absorbed():
    # дальше absorb_m (1.3) это другой объект (ловушки стоят от людей не ближе 2 м)
    f = M.Fusion()
    feed(f, 3, x=0.0)
    feed(f, 3, x=1.6, h=0.6, t0=10.0)
    assert len(f.marks) == 2
    assert sorted(m.status for m in f.marks) == [M.CONFIRMED, M.REJECTED]


def test_a_absorb_can_be_disabled():
    f = M.Fusion({'absorb_m': 0.0})
    feed(f, 3, x=0.0)
    feed(f, 3, x=1.2, h=0.6, t0=10.0)
    assert len(f.marks) == 2


# --- B: пересмотр CONFIRMED ---

def _confirmed_by_close_range(f, x=0.0):
    # робот вплотную: высота 0.3 м (до 0.4 м), по трём замерам - CONFIRMED
    m = feed(f, 3, x=x, rng=0.3, h=0.3, ln=0.3)
    assert m.status == M.CONFIRMED
    return m


def test_b_confirmed_is_revised_when_reliable_heights_are_tall():
    f = M.Fusion()
    m = _confirmed_by_close_range(f)
    for k, r in enumerate([0.6, 1.0, 1.5, 2.0, 2.5, 2.5]):
        m, changed = f.update(0.0, 0.0, 'r1', r, 310.97, 0.55, 1.0, 10.0 + k)
    assert m.status == M.REJECTED and m.reason == 'too_tall+revised'
    # причина из SHADOW_REASONS: детекции того же объекта не рождают новую отметку
    n = len(f.marks)
    feed(f, 3, x=0.9, rng=2.0, h=0.55, t0=30.0)
    assert len(f.marks) == n


def test_b_close_range_readings_never_revise():
    f = M.Fusion()
    m = _confirmed_by_close_range(f)
    for k in range(40):
        m, _ = f.update(0.0, 0.0, 'r1', 0.35, 310.97, 0.55, 0.3, 10.0 + k * 0.2)
    assert m.status == M.CONFIRMED


def test_b_standing_robot_repeats_do_not_revise_person():
    # person_5 seed 1: 25 одинаковых замеров 0.496 с одной точки (0.78 м), потом снова 0.31
    f = M.Fusion()
    m = feed(f, 4, rng=1.4, h=0.31)
    for k in range(25):
        m, _ = f.update(0.0, 0.0, 'r1', 0.78, 310.97, 0.496, 1.5, 10.0 + k * 0.2)
    for k, r in enumerate([1.0, 1.3, 1.6]):
        m, _ = f.update(0.0, 0.0, 'r1', r, 310.97, 0.31, 1.5, 20.0 + k)
    assert m.status == M.CONFIRMED


def test_b_person_with_normal_heights_stays_confirmed():
    f = M.Fusion()
    m = None
    for k in range(60):
        m, _ = f.update(0.0, 0.0, 'r1', 0.6 + 0.04 * k, 310.97, 0.30 + 0.03 * math.sin(k), 1.5, float(k))
    assert m.status == M.CONFIRMED and not any('revised' in h[2] for h in m.history)


# --- C: дальние отметки не сливаются за пределами gate_m ---

def test_c_far_only_mark_is_not_merged_into_neighbour():
    # seed 2 gt: 9 детекций с 9.5 м из тупика слились с отметкой соседа в 1.7 м и человек был потерян
    f = M.Fusion()
    feed(f, 3, x=0.0, rng=2.0)
    far = feed(f, 9, x=1.7, rng=9.5, h=NAN, ln=0.2, t0=10.0)
    assert len(f.marks) == 2
    assert far.status == M.CANDIDATE and far in f.needs_inspection()


def test_c_near_views_of_one_body_still_merge():
    f = M.Fusion()
    feed(f, 3, x=0.0, rng=2.0)
    feed(f, 3, x=1.7, rng=2.5, t0=10.0)
    assert len(f.marks) == 1


def test_c_far_mark_inside_gate_still_joins():
    f = M.Fusion()
    feed(f, 3, x=0.0, rng=2.0)
    feed(f, 3, x=0.8, rng=9.0, h=NAN, t0=10.0)
    assert len(f.marks) == 1


def test_c_air_only_mark_merges_as_before():
    f = M.Fusion(air_robots=['drone_1'])
    feed(f, 3, x=0.0, rng=2.0)
    feed(f, 3, x=1.5, rng=8.0, h=NAN, ln=NAN, t0=10.0, robot='drone_1')
    assert len(f.marks) == 1


# --- D: огонь за человеком греет замеры робота, дрон видит человека холодным ---

def test_d_fire_spot_on_person_lidar_point_does_not_reject_person():
    # seed 3 dev, person_2: у дрона 60 холодных замеров, у робота вблизи один холодный и стоящие горячие (огонь за ним)
    f = M.Fusion(air_robots=['drone_1'])
    feed(f, 10, x=0.0, rng=5.0, h=NAN, ln=NAN, robot='drone_1')
    feed(f, 1, x=0.0, rng=0.9, t0=5.0, robot='g1')
    m = feed(f, 8, x=0.0, rng=0.7, temp=598.81, t0=6.0, robot='g1')
    assert m.status == M.CONFIRMED


def test_d_real_fire_is_still_rejected_when_drone_sees_it_hot():
    f = M.Fusion(air_robots=['drone_1'])
    feed(f, 10, x=0.0, rng=5.0, temp=598.81, h=NAN, ln=NAN, robot='drone_1')
    m = feed(f, 8, x=0.0, rng=0.7, temp=598.81, t0=6.0, robot='g1')
    assert m.status == M.REJECTED


def test_d_air_rule_can_be_disabled():
    f = M.Fusion({'air_temp_min_obs': 0}, air_robots=['drone_1'])
    feed(f, 10, x=0.0, rng=5.0, h=NAN, ln=NAN, robot='drone_1')
    feed(f, 1, x=0.0, rng=0.9, t0=5.0, robot='g1')
    m = feed(f, 8, x=0.0, rng=0.7, temp=598.81, t0=6.0, robot='g1')
    assert m.status == M.REJECTED

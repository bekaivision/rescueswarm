import math

from rs_comms.link_model import (PRESETS, count_walls, in_outage, link_quality,
                                 load_walls, packet_success_prob, parse_outages,
                                 resolve_params)


def wall(cx, cy, sx, sy, yaw=0.0, h=2.0):
    return load_walls({'obstacles': [dict(kind='wall', center=[cx, cy],
                                          size=[sx, sy], yaw=yaw, height=h),
                                     dict(kind='rubble', center=[0, 0],
                                          size=[1, 1], yaw=0, height=1)]})


def test_only_walls_loaded():
    assert len(wall(0, 0, 1, 1)) == 1


def test_cross_2d_ground_blocked():
    w = wall(5, 0, 0.2, 10)
    assert count_walls((0, 0, 0.3), (10, 0, 0.3), w) == 1


def test_drone_above_passes():
    w = wall(5, 0, 0.2, 10, h=2.0)
    assert count_walls((0, 0, 3.0), (10, 0, 3.0), w) == 0


def test_segment_ending_before_wall():
    w = wall(5, 0, 0.2, 10)
    assert count_walls((0, 0, 0.3), (4, 0, 0.3), w) == 0


def test_rotated_wall():
    w = wall(5, 0, 0.2, 10, yaw=math.pi / 4)
    assert count_walls((0, 0, 0.3), (10, 0, 0.3), w) == 1
    # отрезок вдоль повёрнутой стены, но мимо неё
    assert count_walls((0, 8, 0.3), (3, 8, 0.3), w) == 0


def test_sloped_segment_height_rule():
    w = wall(5, 0, 0.2, 10, h=2.0)
    # поднимается: на стене уже выше 2 м
    assert count_walls((0, 0, 0.3), (10, 0, 6.0), w) == 0
    assert count_walls((0, 0, 0.3), (10, 0, 1.0), w) == 1


def test_link_presets():
    P = resolve_params('medium', {})
    up, d_eff, p = link_quality(10.0, 0, P)
    assert up and d_eff == 10.0 and p >= P.loss_base
    up, d_eff, p = link_quality(10.0, 2, P)
    assert d_eff > 10.0 * 2
    assert not up
    up, _, p = link_quality(19.9, 0, P)
    assert up and p > 0.3
    assert not link_quality(20.1, 0, P)[0]


def test_ideal_always_up():
    P = resolve_params('ideal', {})
    up, d, p = link_quality(1e6, 5, P)
    assert up and p == 0.0
    assert packet_success_prob(10 ** 6, p, P) == 1.0


def test_overrides():
    P = resolve_params('good', {'range_m': -1.0, 'delay_s': 0.5, 'x': 1})
    assert P.range_m == 40.0 and P.delay_s == 0.5


def test_packet_prob_monotone():
    P = resolve_params('poor', {})
    a = packet_success_prob(500, 0.2, P)
    b = packet_success_prob(5000, 0.2, P)
    c = packet_success_prob(5000, 0.4, P)
    assert 1.0 >= a > b > c > 0


def test_outages():
    o = parse_outages('ground_1:60:180;drone_2:100:130')
    assert o == [('ground_1', 60.0, 180.0), ('drone_2', 100.0, 130.0)]
    assert parse_outages('') == [] and parse_outages([]) == []
    assert parse_outages(['a:1:2', 'b:3:4']) == [('a', 1.0, 2.0), ('b', 3.0, 4.0)]
    assert in_outage(o, 'ground_1', 60.0) and not in_outage(o, 'ground_1', 180.0)
    assert not in_outage(o, 'drone_2', 50.0)

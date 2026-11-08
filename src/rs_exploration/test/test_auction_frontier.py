import numpy as np

from rs_exploration import auction, frontier


def test_auction_deterministic_and_separated():
    bids = {
        'ground_1': [(1.0, 0.0, 0.0), (5.0, 10.0, 0.0)],
        'ground_2': [(1.2, 0.5, 0.0), (6.0, 20.0, 0.0)],
        'ground_3': [(1.1, 0.2, 0.0), (7.0, -10.0, 0.0)],
    }
    a = auction.assign(bids, 2.5)
    assert a['ground_1'][:2] == (0.0, 0.0)
    assert a['ground_2'][:2] == (20.0, 0.0)
    assert a['ground_3'][:2] == (-10.0, 0.0)
    # одно и то же при другом порядке словаря
    b = auction.assign(dict(reversed(list(bids.items()))), 2.5)
    assert a == b


def test_auction_tie_break_by_robot_id():
    bids = {'ground_2': [(1.0, 0.0, 0.0)], 'ground_1': [(1.0, 0.0, 0.0)]}
    a = auction.assign(bids, 2.0)
    assert list(a) == ['ground_1']


def _room_map():
    d = -np.ones((100, 100), dtype=np.int8)
    d[20:80, 20:60] = 0          # известная свободная область, справа неизвестно
    d[19, 20:60] = 100
    d[80, 20:60] = 100
    d[20:80, 19] = 100
    return d


def test_frontier_on_open_side():
    d = _room_map()
    an = frontier.analyse(d, 0.05, 0.0, 0.0, [0, 0, 5, 5])
    assert len(an.tx) >= 1
    # граница с неизвестным справа: x около 60*0.05 = 3.0
    assert abs(an.tx[0] - 3.0) < 0.2
    cost = frontier.path_costs(an, 1.5, 2.5, an.tx, an.ty)
    assert np.isfinite(cost).all() and cost[0] > 1.0


def test_small_unknown_hole_is_not_frontier():
    d = np.zeros((100, 100), dtype=np.int8)
    d[50, 50] = -1
    an = frontier.analyse(d, 0.05, 0.0, 0.0, [0, 0, 5, 5])
    assert len(an.tx) == 0


def test_auction_inspection_one_robot_per_key_and_priority():
    bids = {
        'ground_1': [(-999.0, 1.0, 0.0, 'mark_1'), (-998.0, 5.0, 5.0, 'mark_2'), (3.0, 0.0, 0.0, '')],
        'ground_2': [(-990.0, 1.5, 0.0, 'mark_1'), (-980.0, 9.0, 9.0, 'mark_3'), (1.0, 20.0, 0.0, '')],
    }
    a = auction.assign(bids, 2.5)
    assert a['ground_1'][:2] == (1.0, 0.0)          # осмотр mark_1 важнее фронтира
    assert a['ground_2'][:2] == (9.0, 9.0)          # mark_1 занят: берёт mark_3


def test_standing_points_face_candidate_with_los():
    from rs_exploration import inspection
    d = np.zeros((100, 100), dtype=np.int8)
    d[:, 50] = 100                                   # стена справа от кандидата
    d[:, 51:] = -1
    an = frontier.analyse(d, 0.05, 0.0, 0.0, [0, 0, 5, 5])
    pts = inspection.standing_points(an, d, 1.5, 2.5)
    assert pts
    for x, y, yaw, _ in pts:
        assert 0.9 <= np.hypot(x - 1.5, y - 2.5) <= 2.1 and x < 2.5
        assert abs(np.arctan2(np.sin(yaw - np.arctan2(2.5 - y, 1.5 - x)), np.cos(yaw - np.arctan2(2.5 - y, 1.5 - x)))) < 1e-6


def test_giveup_noreach_and_open_too_long():
    from rs_exploration.inspection import GiveUp
    g = GiveUp(noreach_s=60.0, max_open_s=300.0)
    # достижимый кандидат ждёт долго, но не осматривается дольше max_open: оставляем по времени
    assert g.update(0.0, ['a', 'b'], {'a'}) == []
    assert g.update(30.0, ['a', 'b'], {'a'}) == []
    out = g.update(61.0, ['a', 'b'], {'a'})
    assert [k for k, _ in out] == ['b']                       # 'b' недостижим 61 с
    # достижимость вернулась: счётчик сброшен
    g2 = GiveUp(noreach_s=60.0, max_open_s=300.0)
    g2.update(0.0, ['b'], set())
    g2.update(50.0, ['b'], {'b'})
    assert g2.update(100.0, ['b'], set()) == []               # заново с 100 с
    assert [k for k, _ in g2.update(161.0, ['b'], set())] == ['b']
    # текущая цель и временная пауза не оставляются по недостижимости
    g3 = GiveUp(noreach_s=10.0, max_open_s=300.0)
    g3.update(0.0, ['c', 'd'], set(), goal_key='c', blocked={'d'})
    assert g3.update(50.0, ['c', 'd'], set(), goal_key='c', blocked={'d'}) == []
    # слишком давно открыт, не цель
    assert [k for k, _ in g3.update(301.0, ['d'], {'d'})] == ['d']
    assert g3.update(301.0, ['c'], {'c'}, goal_key='c') == []

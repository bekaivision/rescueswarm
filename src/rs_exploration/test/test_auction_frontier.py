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

import numpy as np

from rs_exploration import air_targets as at
from rs_exploration import auction

REGION = [0.0, 0.0, 10.0, 8.0]
RES = 0.1


def _unknown_map():
    return -np.ones((80, 100), dtype=np.int8)   # 10 x 8 м, начало в (0, 0)


def test_grid_and_keys():
    assert at.grid_shape(REGION, 2.0) == (5, 4)
    assert at.grid_shape([0, 0, 9.0, 8.0], 2.0) == (5, 4)
    assert at.cell_key(0.1, 0.1, REGION, 2.0) == (0, 0)
    assert at.cell_key(9.9, 7.9, REGION, 2.0) == (4, 3)
    assert at.cell_key(-5, 50, REGION, 2.0) == (0, 3)
    assert at.cell_center((1, 2), REGION, 2.0) == (3.0, 5.0)
    assert at.cell_center((4, 0), [0, 0, 9.0, 8.0], 2.0) == (8.5, 1.0)


def test_unseen_detection_fraction():
    d = _unknown_map()
    assert len(at.unseen_cells(d, RES, 0.0, 0.0, REGION)) == 20
    d[0:20, 0:20] = 0                       # клетка (0, 0) полностью известна
    keys = at.unseen_cells(d, RES, 0.0, 0.0, REGION)
    assert (0, 0) not in keys and len(keys) == 19
    d[20:40, 0:20] = 0
    d[20:26, 0:20] = -1                     # клетка (0, 1): 30 % неизвестно -> не "больше 0.3"
    assert (0, 1) not in at.unseen_cells(d, RES, 0.0, 0.0, REGION, 2.0, 0.3)
    d[20:27, 0:20] = -1                     # 35 % -> не видна
    assert (0, 1) in at.unseen_cells(d, RES, 0.0, 0.0, REGION, 2.0, 0.3)


def test_unseen_outside_map_ignored():
    d = -np.ones((40, 50), dtype=np.int8)   # карта покрывает только 5 x 4 м
    keys = at.unseen_cells(d, RES, 0.0, 0.0, REGION)
    assert len(keys) == 6 and all(i < 3 and j < 2 for i, j in keys)


def test_rank_cost_hysteresis_and_penalty():
    keys = [(0, 0), (4, 3)]
    r = at.rank_cells(keys, REGION, 2.0, (1.0, 1.0))
    assert [e[3] for e in r] == [(0, 0), (4, 3)] and abs(r[0][0]) < 1e-9
    # гистерезис текущей цели
    r = at.rank_cells(keys, REGION, 2.0, (6.0, 4.5), goal_key=(0, 0), hysteresis=100.0)
    assert r[0][3] == (0, 0)
    # штраф за цель другого дрона
    r = at.rank_cells(keys, REGION, 2.0, (1.0, 1.0), others=[(1.5, 1.5)], assign_radius=4.0, other_penalty=50.0)
    assert r[0][3] == (4, 3)
    # облетённые пропускаются
    assert [e[3] for e in at.rank_cells(keys, REGION, 2.0, (1, 1), skip={(0, 0)})] == [(4, 3)]


def test_book_retries_and_failure():
    b = at.CellBook(max_retries=2)
    assert b.on_abort((1, 1)) and b.on_abort((1, 1))
    assert (1, 1) not in b.skip()
    assert not b.on_abort((1, 1))
    assert (1, 1) in b.skip() and (1, 1) in b.failed
    b.on_success((2, 2))
    assert (2, 2) in b.skip() and (2, 2) in b.visited


def test_termination_on_never_known_map():
    """Карта остаётся неизвестной: после облёта всех клеток кандидатов нет, цикл завершается."""
    d = _unknown_map()
    book, tracker = at.CellBook(), at.DoneTracker(3)
    pose = (0.0, 0.0)
    for _ in range(200):
        keys = [k for k in at.unseen_cells(d, RES, 0.0, 0.0, REGION) if k not in book.skip()]
        c = at.rank_cells(keys, REGION, 2.0, pose)
        if tracker.update(len(c)):
            break
        if c:
            book.on_success(c[0][3])
            pose = c[0][1:3]
    assert tracker.empty >= 3 and len(book.visited) == 20


def test_done_tracker_resets():
    t = at.DoneTracker(3)
    assert not t.update(0) and not t.update(0)
    assert not t.update(1)
    assert not t.update(0) and not t.update(0) and t.update(0)
    t2 = at.DoneTracker(1)
    assert not t2.update(0, has_goal=True)


def test_two_drones_goals_not_within_assign_radius():
    d = _unknown_map()
    keys = at.unseen_cells(d, RES, 0.0, 0.0, REGION)
    poses = {'drone_1': (1.0, 1.0), 'drone_2': (1.5, 1.5)}
    table = {}
    for rid, p in poses.items():
        r = at.rank_cells(keys, REGION, 2.0, p)
        table[rid] = [(c, x, y, '') for c, x, y, _ in r[:6]]
    res = auction.assign(table, 4.0)
    assert set(res) == {'drone_1', 'drone_2'}
    (x1, y1, _), (x2, y2, _) = res['drone_1'], res['drone_2']
    assert np.hypot(x1 - x2, y1 - y2) >= 4.0


def test_goal_timeout():
    assert at.goal_timeout(0.0, 2.0) == 20.0
    assert at.goal_timeout(40.0, 2.0) == 50.0

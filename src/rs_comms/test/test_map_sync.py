import zlib

import numpy as np

from rs_comms import map_sync as ms


def _unz(b):
    return np.frombuffer(zlib.decompress(b), dtype=np.int8)


def _src(n=100):
    rng = np.random.default_rng(1)
    src = ms.SourceLayer(n, 0)
    snaps = []
    q = np.full(n, ms.UNSEEN, dtype=np.int8)
    for v in range(1, 6):
        idx = rng.choice(n, 15, replace=False)
        q = q.copy()
        q[idx] = rng.integers(-100, 101, 15)
        src.apply_full(q, v)
        snaps.append(q.copy())
    return src, snaps


def test_full_then_deltas_round_trip():
    src, _ = _src()
    dst = ms.SourceLayer(src.n)
    d, n = src.make_delta(0)
    assert n == int((src.q != ms.UNSEEN).sum())
    dst.apply_delta(_unz(d), 0, src.have)
    assert (dst.q == src.q).all() and dst.have == src.have
    # инкремент
    q2 = src.q.copy()
    q2[3] = 42
    src.apply_full(q2, 6)
    d, n = src.make_delta(5)
    assert n == 1
    dst.apply_delta(_unz(d), 5, 6)
    assert (dst.q == src.q).all() and dst.have == 6


def test_out_of_order_never_overwrites_newer():
    a = ms.SourceLayer(4)
    a.apply_full(np.array([10, 10, 10, 10], dtype=np.int8), 1)
    a.apply_full(np.array([20, 10, 10, 10], dtype=np.int8), 2)
    d2, _ = a.make_delta(1)
    d1 = np.array([5, 5, 5, ms.NOCHANGE], dtype=np.int8)
    p = ms.SourceLayer(4)
    p.apply_delta(_unz(d2), 1, 2)          # новая раньше старой
    p.apply_delta(d1, 0, 1)
    assert p.q[0] == 20                    # новое не затёрто
    assert p.q[1] == 5 and p.q[2] == 5     # то, чего не было, заполнено


def test_gap_does_not_advance_have():
    p = ms.SourceLayer(4)
    p.apply_delta(np.array([1, ms.NOCHANGE, ms.NOCHANGE, ms.NOCHANGE], dtype=np.int8), 0, 1)
    assert p.have == 1
    p.apply_delta(np.array([ms.NOCHANGE, 2, ms.NOCHANGE, ms.NOCHANGE], dtype=np.int8), 3, 4)  # пропуск 2..3
    assert p.have == 1 and p.q[1] == 2
    p.apply_delta(np.array([ms.NOCHANGE, ms.NOCHANGE, 3, ms.NOCHANGE], dtype=np.int8), 1, 3)
    assert p.have == 3
    p.apply_delta(np.array([ms.NOCHANGE, ms.NOCHANGE, ms.NOCHANGE, 4], dtype=np.int8), 3, 4)
    assert p.have == 4


def test_relay_gives_same_q():
    src, _ = _src()
    x = ms.SourceLayer(src.n)
    d, _ = src.make_delta(0)
    x.apply_delta(_unz(d), 0, src.have)
    p = ms.SourceLayer(src.n)
    d2, _ = x.make_delta(0)
    p.apply_delta(_unz(d2), 0, x.have)
    assert (p.q == src.q).all() and p.have == src.have


def test_delta_excludes_cells_above_have():
    x = ms.SourceLayer(3)
    x.apply_delta(np.array([1, ms.NOCHANGE, ms.NOCHANGE], dtype=np.int8), 0, 1)
    x.apply_delta(np.array([ms.NOCHANGE, 9, ms.NOCHANGE], dtype=np.int8), 5, 6)  # пропуск
    d, n = x.make_delta(0)
    assert n == 1 and _unz(d)[1] == ms.NOCHANGE


def test_encode_decode():
    a = np.array([-128, 0, 5, 127], dtype=np.int8)
    for enc in (ms.ENC_RAW, ms.ENC_ZLIB):
        assert (ms.decode(ms.encode(a, enc), enc) == a).all()


def test_merge_to_grid_fuse():
    a = ms.SourceLayer(4)
    b = ms.SourceLayer(4)
    a.q[:] = [20, 20, -40, ms.UNSEEN]      # 1.0, 1.0, -2.0
    b.q[:] = [20, ms.UNSEEN, -40, ms.UNSEEN]
    L, seen = ms.merge([a, b])
    assert np.allclose(L, [2.0, 1.0, -2.0, 0.0])      # -4.0 обрезано до L_MIN
    assert seen.tolist() == [True, True, True, False]
    g = ms.to_grid(L, seen)
    assert g.tolist() == [100, 100, 0, -1]
    # порог 0.4
    c = ms.SourceLayer(2)
    c.q[:] = [8, 7]
    Lc, sc = ms.merge([c])
    assert ms.to_grid(Lc, sc).tolist() == [100, 0]
    # сумма обрезается сверху
    d = ms.SourceLayer(1)
    d.q[:] = [100]
    assert ms.merge([d, d])[0][0] == ms.L_MAX
    assert ms.merge([]) is None
    ag = np.array([0, 100, 100, 100], dtype=np.int8)
    f = ms.fuse(np.array([-1, 0, 100, -1], dtype=np.int8), np.array([False, True, True, False]), ag)
    assert f.tolist() == [0, 0, 100, 100]


def _p(have, nb):
    return (have, set(nb))


def test_serve_source_direct():
    peers = {'b': _p({'a': 0}, ['a']), 'c': _p({'a': 5}, [])}
    assert ms.serve_targets('a', 'a', 5, peers) == ['b']      # c уже имеет всё


def test_serve_relay_when_source_not_neighbor():
    # x держит s (версия 5), P не слышит s, общего лучшего соседа нет
    peers = {'p': _p({'s': 2}, ['x'])}
    assert ms.serve_targets('x', 's', 5, peers) == ['p']
    # P слышит s сам: не обслуживаем
    peers = {'p': _p({'s': 2}, ['x', 's'])}
    assert ms.serve_targets('x', 's', 5, peers) == []


def test_serve_relay_tiebreak_and_better_peer():
    # y имеет больше, чем x: x не обслуживает
    peers = {'p': _p({'s': 1}, ['x', 'y']), 'y': _p({'s': 7}, ['x', 'p'])}
    assert ms.serve_targets('x', 's', 5, peers) == []
    # поровну: обслуживает тот, чьё имя меньше
    peers = {'p': _p({'s': 1}, ['x', 'y']), 'y': _p({'s': 5}, ['x', 'p'])}
    assert ms.serve_targets('x', 's', 5, peers) == ['p']        # x < y
    peers = {'p': _p({'s': 1}, ['y', 'z']), 'y': _p({'s': 5}, ['z', 'p']), 'z': _p({'s': 5}, ['y', 'p'])}
    assert ms.serve_targets('z', 's', 5, peers) == []           # y < z
    assert ms.serve_targets('y', 's', 5, {'p': peers['p'], 'z': peers['z']}) == ['p']
    # Y не наш сосед: не считается
    peers = {'p': _p({'s': 1}, ['x', 'y'])}
    assert ms.serve_targets('x', 's', 5, peers) == ['p']

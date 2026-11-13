from rs_comms.obs_log import ObsLog


def test_in_order_delivery_with_gaps_and_duplicates():
    lg = ObsLog()
    assert lg.add('a', 2, b'2') == []
    assert lg.have('a') == 0
    assert lg.add('a', 3, b'3') == []
    assert lg.add('a', 1, b'1') == [(1, b'1'), (2, b'2'), (3, b'3')]
    assert lg.have('a') == 3
    assert lg.add('a', 2, b'2') == []                 # дубликат
    assert lg.add('a', 5, b'5') == []
    assert lg.have('a') == 3
    assert lg.add('a', 4, b'4') == [(4, b'4'), (5, b'5')]
    assert lg.have('a') == 5
    assert lg.next_deliver['a'] == 6


def test_sources_and_get_range():
    lg = ObsLog()
    for i in (1, 2, 4):
        lg.add('a', i, bytes([i]))
    lg.add('b', 1, b'x')
    assert sorted(lg.sources()) == ['a', 'b']
    assert lg.have('zz') == 0
    assert lg.get_range('a', 1, 4) == [(2, b'\x02'), (4, b'\x04')]
    assert lg.get_range('a', 0, 1) == [(1, b'\x01')]
    assert lg.get_range('nope', 0, 5) == []

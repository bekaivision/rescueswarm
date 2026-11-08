from rs_perception import marks as M


def feed(f, n, x=1.0, y=2.0, rng=2.0, temp=310.0, h=0.2, ln=1.5):
    for i in range(n):
        m, _ = f.update(x, y, 'r1', rng, temp, h, ln, float(i))
    return m


def test_person_confirmed():
    f = M.Fusion()
    assert feed(f, 3, ln=1.5).status == M.CONFIRMED


def test_far_stays_candidate():
    f = M.Fusion()
    m = feed(f, 10, rng=6.0)
    assert m.status == M.CANDIDATE and f.needs_inspection() == [m]


def test_hot_rejected():
    assert feed(M.Fusion(), 3, temp=335.0).status == M.REJECTED


def test_tall_debris_rejected():
    assert feed(M.Fusion(), 3, temp=309.0, h=0.5, ln=0.7).status == M.REJECTED


def test_gate_merges():
    f = M.Fusion()
    feed(f, 1, x=1.0)
    feed(f, 1, x=1.5)
    assert len(f.marks) == 1


def test_merge_close_marks():
    f = M.Fusion()
    feed(f, 1, x=0.0)
    feed(f, 1, x=2.0)          # вне тела (1.8 м) - отдельная отметка
    assert len(f.marks) == 2
    feed(f, 3, x=1.0, ln=1.5)  # между ними: сливает обе
    assert len(f.marks) == 1


def test_negative_evidence_rejects_ghost_and_keeps_seen():
    import math
    from rs_perception import marks as M
    f = M.Fusion()
    f.update(2.0, 0.0, 'ground_1', 6.0, 310.9, float('nan'), float('nan'), 0.0)
    f.update(2.0, 0.05, 'ground_1', 6.0, 310.9, float('nan'), float('nan'), 0.1)
    m = f.marks[0]
    assert m.status == M.CANDIDATE
    t = 1.0
    out = []
    while t < 6.0 and not out:
        out = f.observe_view('ground_1', 0.0, 0.0, 0.0, t)   # цель в 2 м прямо по курсу (камера +0.25 м)
        t += 0.2
    assert out and m.status == M.REJECTED and m.reason == 'not_seen_close'
    # детекции сбрасывают накопление
    f2 = M.Fusion()
    f2.update(2.0, 0.0, 'ground_1', 6.0, 310.9, float('nan'), float('nan'), 0.0)
    f2.update(2.0, 0.0, 'ground_1', 6.0, 310.9, float('nan'), float('nan'), 0.1)
    for k in range(40):
        f2.observe_view('ground_1', 0.0, 0.0, 0.0, 1.0 + 0.2 * k)
        if k % 5 == 0:
            f2.update(2.0, 0.0, 'ground_1', 2.0, 310.9, float('nan'), float('nan'), 1.0 + 0.2 * k)
    assert f2.marks[0].status != M.REJECTED


def test_body_ends_merge_into_one_person():
    f = M.Fusion()
    feed(f, 3, x=0.0, y=0.0)                 # один конец тела
    feed(f, 3, x=1.5, y=0.0)                 # другой конец, дальше gate_m (1.0)
    assert len(f.marks) == 1 and f.marks[0].status == M.CONFIRMED
    assert 0.5 < f.marks[0].x < 1.0          # положение стало между концами


def test_body_merge_not_beyond_body_length_or_for_other_temperature():
    f = M.Fusion()
    feed(f, 3, x=0.0)
    feed(f, 3, x=2.2)                        # дальше длины тела: два человека
    assert len(f.marks) == 2
    g = M.Fusion()
    feed(g, 3, x=0.0, temp=310.97)
    feed(g, 3, x=1.5, temp=308.4, h=0.2)     # тёплая ловушка рядом: другая температура
    assert len(g.marks) == 2


def test_body_merge_skips_rejected_and_tall():
    f = M.Fusion()
    feed(f, 3, x=0.0, temp=309.0, h=0.5, ln=0.7)   # REJECTED (высокая ловушка)
    feed(f, 3, x=1.5, temp=309.0, h=0.2)
    assert len(f.marks) == 2

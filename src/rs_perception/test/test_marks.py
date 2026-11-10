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


def test_single_hot_outlier_does_not_reject_person():
    # 7 замеров человека (311 K) и один замер огня (599 K): среднее выше 320 K, медиана 311 K
    f = M.Fusion()
    feed(f, 7, temp=311.0, rng=4.0)
    m, _ = f.update(1.0, 2.0, 'r1', 4.0, 599.0, 0.2, 1.5, 8.0)
    assert m.status != M.REJECTED


def test_mostly_hot_rejected_with_cold_outlier():
    f = M.Fusion()
    feed(f, 5, temp=599.0, rng=4.0)
    m, _ = f.update(1.0, 2.0, 'r1', 4.0, 311.0, 0.2, 1.5, 6.0)
    assert m.status == M.REJECTED and m.reason == 'too_hot'


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


def test_detection_near_rejected_debris_joins_it():
    # ловушка отвергнута как слишком высокая; сдвинутые ошибкой позы (1.2 м) низкие замеры не дают новой отметки
    f = M.Fusion()
    m0 = feed(f, 6, x=0.0, y=0.0, temp=309.0, h=0.5)
    assert m0.status == M.REJECTED and m0.reason == 'too_tall'
    m1 = feed(f, 8, x=1.2, y=0.0, temp=309.0, h=0.25)
    assert m1 is m0 and m1.status == M.REJECTED
    assert not any(m.status == M.CONFIRMED for m in f.marks)


def test_person_two_metres_from_rejected_debris_confirmed():
    # человек не ближе 2 м от ловушки (generate.py): тень отвергнутой отметки его не глотает
    f = M.Fusion()
    feed(f, 6, x=0.0, y=0.0, temp=309.0, h=0.5)
    m = feed(f, 8, x=2.0, y=0.0, temp=310.0, h=0.2)
    assert m.status == M.CONFIRMED


def test_not_seen_close_does_not_shadow():
    # отметка, отвергнутая как "не увидели вблизи" (призрак), не глотает соседние детекции
    f = M.Fusion()
    m0 = feed(f, 2, x=0.0, y=0.0, rng=6.0)
    f._set(m0, M.REJECTED, 1.0, 'not_seen_close')
    m = feed(f, 8, x=1.2, y=0.0, temp=310.0, h=0.2)
    assert m is not m0 and m.status == M.CONFIRMED


def test_few_height_samples_do_not_confirm():
    f = M.Fusion()
    # 3 близких вида, высота известна только в двух: ждём третий замер высоты
    f.update(1.0, 2.0, 'r1', 2.0, 310.0, 0.2, 1.5, 0.0)
    f.update(1.0, 2.0, 'r1', 2.0, 310.0, 0.2, 1.5, 0.1)
    m, _ = f.update(1.0, 2.0, 'r1', 2.0, 310.0, float('nan'), 1.5, 0.2)
    assert m.status == M.CANDIDATE
    m = feed(f, 1, temp=310.0, h=0.2)
    assert m.status == M.CONFIRMED


def test_fire_outliers_do_not_push_person_out_of_band():
    # человек 311 K, рядом огонь: 5 замеров 599 K из 20 не делают отметку temp_out_of_band (seed 2 slam)
    f = M.Fusion()
    for i in range(15):
        f.update(1.0, 2.0, 'r1', 2.0, 311.0, float('nan'), 1.5, float(i))
    for i in range(5):
        m, _ = f.update(1.0, 2.0, 'r1', 2.0, 599.0, float('nan'), 1.5, 20.0 + i)
    assert m.status == M.CANDIDATE
    m = feed(f, 3, temp=311.0, h=0.25)
    assert m.status == M.CONFIRMED

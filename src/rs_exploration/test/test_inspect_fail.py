"""Правило неудач осмотра: кулдаун после отказа и сбой сразу после отправки цели (без ROS)."""
from rs_exploration import inspection


def test_quick_abort_is_transient():
    assert inspection.is_transient_fail(0.6, 2.0)
    assert not inspection.is_transient_fail(2.0, 2.0)
    assert not inspection.is_transient_fail(10.0, 2.0)


def test_transient_not_counted():
    assert inspection.count_fail(0, True) == 0
    assert inspection.count_fail(1, True) == 1
    assert inspection.count_fail(0, False) == 1
    assert inspection.count_fail(1, False) == 2


def test_two_quick_aborts_do_not_give_up():
    n = 0
    for _ in range(5):
        n = inspection.count_fail(n, inspection.is_transient_fail(0.6, 2.0))
    assert n == 0


def test_blocked_window():
    block = {'mark_4@-4.8,-0.7': 75.0}
    assert inspection.blocked(block, 'mark_4@-4.8,-0.7', 60.0)
    assert not inspection.blocked(block, 'mark_4@-4.8,-0.7', 75.0)
    assert not inspection.blocked(block, 'mark_5', 60.0)

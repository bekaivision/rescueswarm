"""Решение "ждать синхронизации" в режиме связи (чистая функция) и узел coverage с comms=true."""
from rs_eval.coverage import sync_wait_decision


def test_synced():
    assert sync_wait_decision(0.80, 0.802, 3.0, 0.005, 120.0) == (True, 'synced')
    assert sync_wait_decision(0.85, 0.80, 0.0, 0.005, 120.0) == (True, 'synced')


def test_waiting_then_timeout():
    assert sync_wait_decision(0.70, 0.80, 10.0, 0.005, 120.0) == (False, 'waiting')
    assert sync_wait_decision(0.70, 0.80, 120.0, 0.005, 120.0) == (True, 'timeout')

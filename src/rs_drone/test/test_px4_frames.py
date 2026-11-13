import math

import pytest

from rs_drone import px4_frames as F


def test_position_north_east_down():
    # 3 м на север, 1 м на восток, 2 м вверх от точки старта (10, 20, 0.2)
    assert F.ned_to_world((3.0, 1.0, -2.0), (10.0, 20.0, 0.2)) == pytest.approx((11.0, 23.0, 2.2))


def test_velocity_roundtrip():
    v = (0.5, -1.0, 0.3)
    assert F.vel_ned_to_enu(F.vel_enu_to_ned(v)) == pytest.approx(v)
    # на восток в ENU = y в NED
    assert F.vel_enu_to_ned((1.0, 0.0, 0.0)) == pytest.approx((0.0, 1.0, -0.0))


@pytest.mark.parametrize('yaw_ned,yaw_enu', [(0.0, math.pi / 2), (math.pi / 2, 0.0), (math.pi, -math.pi / 2)])
def test_heading(yaw_ned, yaw_enu):
    q = F.euler_to_quat(0.0, 0.0, yaw_ned)
    r, p, y = F.quat_to_euler(*F.q_ned_frd_to_enu_flu(q))
    assert r == pytest.approx(0.0, abs=1e-9) and p == pytest.approx(0.0, abs=1e-9)
    assert F.wrap(y - yaw_enu) == pytest.approx(0.0, abs=1e-9)
    assert F.wrap(F.yaw_enu_to_ned(yaw_enu) - yaw_ned) == pytest.approx(0.0, abs=1e-9)


def test_pitch_nose_up_keeps_meaning():
    # нос вверх в FRD/NED - положительный тангаж; во FLU/ENU нос вверх - отрицательный тангаж
    q = F.euler_to_quat(0.0, 0.2, 0.0)
    _, p, _ = F.quat_to_euler(*F.q_ned_frd_to_enu_flu(q))
    assert p == pytest.approx(-0.2)


def test_euler_roundtrip():
    e = (0.1, -0.2, 2.5)
    assert F.quat_to_euler(*F.euler_to_quat(*e)) == pytest.approx(e)

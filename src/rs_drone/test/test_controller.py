import math

import pytest

from rs_drone import controller as c


def test_clip_norm():
    assert c.clip_norm(3.0, 4.0, 10.0) == (3.0, 4.0)
    x, y = c.clip_norm(3.0, 4.0, 2.5)
    assert math.hypot(x, y) == pytest.approx(2.5)
    assert x / y == pytest.approx(0.75)


def test_approach_speed_profile():
    assert c.approach_speed(0.0, 2.0, 1.0, 1.2) == 0.0
    assert c.approach_speed(100.0, 2.0, 1.0, 1.2) == 2.0          # крейсерская
    assert c.approach_speed(0.5, 2.0, 1.0, 1.2) == pytest.approx(0.6)   # kp * d
    # торможение: на расстоянии, равном v^2 / (2a), скорость не выше крейсерской
    assert c.approach_speed(1.5, 2.0, 1.0, 10.0) == pytest.approx(math.sqrt(3.0))


def test_desired_velocity_direction():
    vx, vy = c.desired_velocity_xy(0.0, -10.0, 2.0)
    assert vx == 0.0 and vy == pytest.approx(-2.0)
    assert c.desired_velocity_xy(0.0, 0.0, 2.0) == (0.0, 0.0)


def test_slew_limits_change():
    assert c.slew(0.0, 5.0, 2.0, 0.1) == pytest.approx(0.2)
    assert c.slew(1.0, -5.0, 2.0, 0.1) == pytest.approx(0.8)
    assert c.slew(1.0, 1.05, 2.0, 0.1) == pytest.approx(1.05)
    x, y = c.slew_xy((0.0, 0.0), (3.0, 4.0), 2.0, 0.1)
    assert math.hypot(x, y) == pytest.approx(0.2)


def simulate(start, target, v_xy=2.0, vz=1.0, a_xy=1.5, a_z=1.0, dt=0.05, t_max=120.0):
    pos, cmd = list(start), (0.0, 0.0, 0.0)
    max_acc, max_v, max_vz, t = 0.0, 0.0, 0.0, 0.0
    while t < t_max:
        new = c.velocity_command(cmd, pos, target, v_xy, vz, a_xy, a_z, dt)
        max_acc = max(max_acc, math.hypot(new[0] - cmd[0], new[1] - cmd[1]) / dt)
        max_v = max(max_v, math.hypot(new[0], new[1]))
        max_vz = max(max_vz, abs(new[2]))
        cmd = new
        for i in range(3):
            pos[i] += cmd[i] * dt
        t += dt
        if math.dist(pos, target) < 0.02 and math.hypot(*cmd) < 0.05:
            break
    return pos, t, max_acc, max_v, max_vz


def test_velocity_command_reaches_target_within_limits():
    pos, t, max_acc, max_v, max_vz = simulate((0.0, 0.0, 0.0), (12.0, -9.0, 4.5))
    assert math.dist(pos, (12.0, -9.0, 4.5)) < 0.05
    assert max_v <= 2.0 + 1e-9
    assert max_vz <= 1.0 + 1e-9
    assert max_acc <= 1.5 + 1e-6
    assert t < 15.0 / 2.0 * 3 + 20.0       # быстрее таймаута цели


def test_no_overshoot_xy():
    # движение по прямой: расстояние до цели не должно расти после начала торможения
    pos, cmd, dt = [0.0, 0.0, 4.5], (0.0, 0.0, 0.0), 0.05
    target = (10.0, 0.0, 4.5)
    for _ in range(1200):
        cmd = c.velocity_command(cmd, pos, target, 2.0, 1.0, 1.5, 1.0, dt)
        pos[0] += cmd[0] * dt
        assert pos[0] <= 10.0 + 0.05


def test_goal_timeout_and_reached():
    assert c.goal_timeout(30.0, 2.0) == pytest.approx(65.0)
    assert c.goal_reached(0.3, 0.1)
    assert not c.goal_reached(0.3, 0.5)     # слишком быстро
    assert not c.goal_reached(0.8, 0.0)     # слишком далеко


def test_at_altitude():
    assert c.at_altitude(4.45, 4.5, 0.05)
    assert not c.at_altitude(3.0, 4.5, 0.0)
    assert not c.at_altitude(4.5, 4.5, 0.8)


def test_frames():
    vx, vy = c.world_to_body_xy(1.0, 0.0, math.pi / 2)
    assert (vx, vy) == pytest.approx((0.0, -1.0))
    assert c.yaw_from_quat(0.0, 0.0, math.sin(0.4), math.cos(0.4)) == pytest.approx(0.8)


def test_velocity_estimator():
    e = c.VelocityEstimator(alpha=1.0)
    e.update(0.0, (0.0, 0.0, 0.0))
    v = e.update(0.5, (1.0, 0.0, -0.5))
    assert v == pytest.approx((2.0, 0.0, -1.0))

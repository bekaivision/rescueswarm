"""Математика управления дроном: чистые функции без ROS (проверяются юнит-тестами).

Дрон управляется командой скорости (vx, vy, vz) в мировой системе. Последняя отданная команда
считается текущей скоростью: ограничение ускорения применяется к ней (slew).
Профиль подхода: скорость не больше крейсерской, не больше kp * расстояние и не больше
sqrt(2 * a * расстояние) (чтобы успеть затормозить).
"""
import math


def clip_norm(vx, vy, vmax):
    """Ограничить длину вектора (vx, vy) значением vmax."""
    n = math.hypot(vx, vy)
    if n <= vmax or n < 1e-12:
        return vx, vy
    k = vmax / n
    return vx * k, vy * k


def clip(v, lo, hi):
    return max(lo, min(hi, v))


def approach_speed(dist, v_max, a_brake, kp):
    """Желаемая скорость на расстоянии dist до цели."""
    if dist <= 0.0:
        return 0.0
    return min(v_max, kp * dist, math.sqrt(2.0 * a_brake * dist))


def desired_velocity_xy(err_x, err_y, v_max, a_brake=1.0, kp=1.2):
    """Желаемая скорость к цели по вектору ошибки (цель - позиция)."""
    d = math.hypot(err_x, err_y)
    if d < 1e-9:
        return 0.0, 0.0
    s = approach_speed(d, v_max, a_brake, kp)
    return err_x / d * s, err_y / d * s


def desired_vz(err_z, vz_max=1.0, kp=1.0):
    return clip(kp * err_z, -vz_max, vz_max)


def slew(cur, des, a_max, dt):
    """Приблизить cur к des так, чтобы изменение не превысило a_max * dt (одна ось)."""
    step = a_max * dt
    return cur + clip(des - cur, -step, step)


def slew_xy(cur, des, a_max, dt):
    """То же для вектора (vx, vy): длина изменения не больше a_max * dt."""
    dx, dy = des[0] - cur[0], des[1] - cur[1]
    dx, dy = clip_norm(dx, dy, a_max * dt)
    return cur[0] + dx, cur[1] + dy


def velocity_command(cmd_prev, pos, target, v_xy_max, vz_max, a_xy, a_z, dt,
                     a_brake=1.0, kp_xy=1.2, kp_z=1.0):
    """Один шаг регулятора: новая команда (vx, vy, vz) к точке target = (x, y, z).

    cmd_prev - предыдущая команда, pos - текущая позиция. Результат ограничен по скорости
    (xy <= v_xy_max, |z| <= vz_max) и по ускорению (a_xy, a_z).
    """
    dvx, dvy = desired_velocity_xy(target[0] - pos[0], target[1] - pos[1], v_xy_max, a_brake, kp_xy)
    dvz = desired_vz(target[2] - pos[2], vz_max, kp_z)
    vx, vy = slew_xy((cmd_prev[0], cmd_prev[1]), (dvx, dvy), a_xy, dt)
    vz = slew(cmd_prev[2], dvz, a_z, dt)
    vx, vy = clip_norm(vx, vy, v_xy_max)
    return vx, vy, clip(vz, -vz_max, vz_max)


def goal_timeout(distance, speed, k=3.0, extra=20.0):
    """Время на цель, с: расстояние / скорость * 3 + 20."""
    return distance / max(speed, 1e-3) * k + extra


def goal_reached(dist_xy, speed_xy, tol_xy=0.5, tol_speed=0.3):
    """Цель достигнута: ближе tol_xy по горизонтали и скорость ниже tol_speed."""
    return dist_xy <= tol_xy and speed_xy <= tol_speed


def at_altitude(z, z_target, vz, tol=0.15, tol_vz=0.25):
    return abs(z - z_target) <= tol and abs(vz) <= tol_vz


def world_to_body_xy(vx, vy, yaw):
    """Повернуть вектор из мировой системы в систему корпуса (курс yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx + s * vy, -s * vx + c * vy


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class VelocityEstimator:
    """Скорость конечной разностью по позам (эксп. сглаживание), если сенсор скорости недоступен."""

    def __init__(self, alpha=0.5):
        self.alpha = alpha
        self.t = None
        self.p = None
        self.v = (0.0, 0.0, 0.0)

    def update(self, t, p):
        if self.t is not None and t > self.t:
            dt = t - self.t
            raw = [(p[i] - self.p[i]) / dt for i in range(3)]
            a = self.alpha
            self.v = tuple(a * raw[i] + (1.0 - a) * self.v[i] for i in range(3))
        self.t, self.p = t, tuple(p)
        return self.v

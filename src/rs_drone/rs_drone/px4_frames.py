"""Перевод систем координат PX4 в мировую систему Gazebo (чистые функции, без ROS).

PX4: локальная система NED (x на север, y на восток, z вниз), начало там, где EKF стартовал (точка
взлёта); корпус FRD (x вперёд, y вправо, z вниз). Мир Gazebo с <spherical_coordinates> ENU:
x на восток, y на север, z вверх; корпус FLU.
Положение: east = y_ned, north = x_ned, up = -z_ned, затем сдвиг на точку старта в мире.
Ориентация: крен тот же, тангаж и рыскание меняют знак, рыскание ENU = pi/2 - рыскание NED.
"""
import math


def quat_to_euler(w, x, y, z):
    """Кватернион (w, x, y, z) -> (крен, тангаж, рыскание), порядок ZYX."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return roll, pitch, math.atan2(siny, cosy)


def euler_to_quat(roll, pitch, yaw):
    """(крен, тангаж, рыскание) -> кватернион (w, x, y, z), порядок ZYX."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def ned_to_world(pos_ned, origin_world):
    """Положение NED (относительно точки старта) -> мир ENU."""
    n, e, d = pos_ned
    ox, oy, oz = origin_world
    return ox + e, oy + n, oz - d


def vel_ned_to_enu(v_ned):
    n, e, d = v_ned
    return e, n, -d


def vel_enu_to_ned(v_enu):
    x, y, z = v_enu
    return y, x, -z


def q_ned_frd_to_enu_flu(q):
    """Ориентация корпуса: кватернион PX4 (w, x, y, z), FRD в NED -> FLU в ENU."""
    roll, pitch, yaw = quat_to_euler(*q)
    return euler_to_quat(roll, -pitch, wrap(math.pi / 2 - yaw))


def yaw_enu_to_ned(yaw_enu):
    return wrap(math.pi / 2 - yaw_enu)

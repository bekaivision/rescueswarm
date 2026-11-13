"""Геометрия дрона (без ROS): луч через пиксель тепловизора до пола z = 0 в мире."""
import numpy as np


def pixel_floor_hit(u, v, K, T_world_cam, floor_z=0.0):
    """Луч через пиксель (u, v) и пересечение с плоскостью z = floor_z в мире.

    K = (fx, fy, cx, cy). Кадр камеры Gazebo: x по оптической оси, y влево, z вверх.
    T_world_cam: 4x4 мир <- камера (полная поза, камера может быть наклонена).
    Возвращает (точка в мире (3,), наклонная дальность камера-точка) или None, если луч
    не идёт вниз или не достигает пола перед камерой.
    """
    fx, fy, cx, cy = K
    d_cam = np.array([1.0, -(u - cx) / fx, -(v - cy) / fy])
    R, t = T_world_cam[:3, :3], T_world_cam[:3, 3]
    d = R @ d_cam
    if d[2] > -1e-6:
        return None
    s = (floor_z - t[2]) / d[2]
    if s <= 0.0:
        return None
    p = t + s * d
    return p, float(s * np.linalg.norm(d_cam))


def touches_border(blob, w, h):
    """Пятно упирается в край кадра: обрезано, центр ненадёжен."""
    return blob['u0'] <= 0 or blob['v0'] <= 0 or blob['u1'] >= w - 1 or blob['v1'] >= h - 1

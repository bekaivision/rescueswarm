"""Виртуальный 2D-скан «только другие роботы» (ядро без ROS).

Для робота i по позам остальных (x, y, yaw) считаем дальности лучей от его центра до
прямоугольных корпусов других. Луч без попадания = inf (так costmap «стирает» старые метки).
Так роботы видят друг друга, хотя настоящий лидар (z ~ 0.28 м) их низкие корпуса не видит.
"""
import numpy as np


def make_angles(n_rays=720):
    """Углы лучей в кадре робота, от -pi (шаг 2*pi/n_rays)."""
    return -np.pi + np.arange(n_rays) * (2.0 * np.pi / n_rays)


def ray_rect_ranges(angles, me, others, half_len=0.25, half_wid=0.15, pad=0.03, max_range=8.0):
    """Дальность до ближайшего корпуса для каждого луча.

    angles: углы лучей в кадре робота me; me = (x, y, yaw); others = [(x, y, yaw), ...] в мире;
    корпус = прямоугольник 2*half_len x 2*half_wid, увеличенный на pad с каждой стороны.
    Возвращает массив дальностей (inf, если луч ничего не встретил в пределах max_range).
    """
    res = np.full(angles.shape, np.inf)
    mx, my, myaw = me
    wa = angles + myaw                      # направления лучей в мире
    dx_w, dy_w = np.cos(wa), np.sin(wa)
    hl, hw = half_len + pad, half_wid + pad
    for ox, oy, oyaw in others:
        c, s = np.cos(oyaw), np.sin(oyaw)
        # начало и направления лучей в системе координат другого робота
        px, py = mx - ox, my - oy
        lx, ly = c * px + s * py, -s * px + c * py
        dx, dy = c * dx_w + s * dy_w, -s * dx_w + c * dy_w
        # пересечение луча с прямоугольником (метод плоскостей); центр me вне прямоугольника
        with np.errstate(divide='ignore', invalid='ignore'):
            tx1, tx2 = (-hl - lx) / dx, (hl - lx) / dx
            ty1, ty2 = (-hw - ly) / dy, (hw - ly) / dy
        tmin = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
        tmax = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
        # луч параллелен стороне и лежит снаружи: деление даёт nan/inf, это промах
        if abs(lx) <= hl and abs(ly) <= hw:
            tmin = np.zeros_like(tmin)       # центр внутри чужого корпуса: максимальная близость
            tmax = np.full_like(tmax, 1.0)
        hit = (tmax >= tmin) & (tmax > 0) & np.isfinite(tmin)
        t = np.where(hit, np.maximum(tmin, 0.0), np.inf)
        res = np.minimum(res, t)
    res[res > max_range] = np.inf
    return res

"""Слои карты по источникам (Фаза 6): квантование log-odds в int8 и слияние слоёв.

Кодирование клетки (как в MapLayer.data): -128 = источник клетку не видел, иначе
q = clip(round(logodds * 20), -100, 100). Слияние источников одного типа:
L = clip(сумма q/20 по источникам, видевшим клетку, -2.0, 3.5), видно = любой видел;
значение карты: -1 не видно, 100 если L >= 0.4, иначе 0. Общая карта = наземное значение там,
где клетку видел хоть один наземный источник, иначе воздушное (как air_update.fuse).
"""
import numpy as np

UNSEEN = -128
UNCHANGED = 127
SCALE = 20.0
L_MIN = -2.0
L_MAX = 3.5
OCC_THR = 0.4


def quantize(logodds, seen):
    """int8 на клетку: -128 не видно, иначе clip(round(logodds * 20), -100, 100)."""
    return np.where(seen, np.clip(np.round(logodds * SCALE), -100, 100), UNSEEN).astype(np.int8)


def merge_layers(qs):
    """Слияние квантованных слоёв одного типа -> (L float32, seen bool). Пустой список -> (None, None)."""
    if not qs:
        return None, None
    total = np.zeros(qs[0].shape, dtype=np.float32)
    seen = np.zeros(qs[0].shape, dtype=bool)
    for q in qs:
        s = q != UNSEEN
        total += np.where(s, q.astype(np.float32) / SCALE, 0.0)
        seen |= s
    return np.clip(total, L_MIN, L_MAX), seen


def to_grid(L, seen):
    """Значения OccupancyGrid (int8): -1 не видно, 100 занято, 0 свободно."""
    d = np.full(L.shape, -1, dtype=np.int8)
    d[seen] = 0
    d[seen & (L >= OCC_THR)] = 100
    return d


def merge_grid(qs):
    """Слияние слоёв одного типа сразу в значения карты; пустой список -> None."""
    L, seen = merge_layers(qs)
    return None if L is None else to_grid(L, seen)


def fuse_grids(ground, air):
    """Общая карта: наземное значение там, где его видели, иначе воздушное. Любой аргумент может быть None."""
    if ground is None and air is None:
        return None
    if ground is None:
        return air.copy()
    if air is None:
        return ground.copy()
    return np.where(ground >= 0, ground, air).astype(np.int8)

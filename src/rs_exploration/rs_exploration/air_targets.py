"""Выбор целей для дронов (чистые функции на numpy, без ROS): непросмотренные сверху участки.

Область поиска режется на крупные клетки (cell_m, по умолчанию 2 м). Крупная клетка "не видна",
если неизвестных мелких клеток /map_air в ней больше доли unseen_frac. Цель дрона = центр такой клетки,
которую он ещё не облетел и не провалил. Цена = расстояние по прямой (дрон летит над стенами)
минус бонус текущей цели плюс штраф за близость к цели другого дрона.

Клетка считается облетённой, когда действие NavigateToPose завершилось SUCCEEDED: её больше не берём,
даже если на карте там ещё остались неизвестные клетки. Это гарантирует завершение.
"""
import math

import numpy as np


def grid_shape(region, cell_m):
    """Число крупных клеток (nx, ny) в области region = [xmin, ymin, xmax, ymax]."""
    xmin, ymin, xmax, ymax = region
    return (max(1, int(math.ceil((xmax - xmin) / cell_m - 1e-9))),
            max(1, int(math.ceil((ymax - ymin) / cell_m - 1e-9))))


def cell_key(x, y, region, cell_m):
    """Индексы (i, j) крупной клетки, в которую попадает точка (вне области: ближайшая крупная клетка)."""
    nx, ny = grid_shape(region, cell_m)
    i = int(math.floor((x - region[0]) / cell_m))
    j = int(math.floor((y - region[1]) / cell_m))
    return (min(max(i, 0), nx - 1), min(max(j, 0), ny - 1))


def cell_center(key, region, cell_m):
    """Центр крупной клетки (последняя клетка у края области может быть уже: центр по её фактической ширине)."""
    xmin, ymin, xmax, ymax = region
    x0, y0 = xmin + key[0] * cell_m, ymin + key[1] * cell_m
    x1, y1 = min(x0 + cell_m, xmax), min(y0 + cell_m, ymax)
    return (0.5 * (x0 + x1), 0.5 * (y0 + y1))


def unseen_cells(data, res, ox, oy, region, cell_m=2.0, unseen_frac=0.3):
    """Крупные клетки, где неизвестных мелких клеток больше unseen_frac.

    data: int8 (h, w), строка 0 = низ карты, -1 = неизвестно. Возвращает список ключей (i, j).
    Клетки целиком вне карты (нет ни одной мелкой клетки) считаются просмотренными: идти туда нечего.
    """
    h, w = data.shape
    nx, ny = grid_shape(region, cell_m)
    unk = (data < 0).astype(np.int64)
    integ = np.zeros((h + 1, w + 1), dtype=np.int64)
    integ[1:, 1:] = unk.cumsum(0).cumsum(1)
    out = []
    for j in range(ny):
        for i in range(nx):
            x0, y0 = region[0] + i * cell_m, region[1] + j * cell_m
            x1, y1 = min(x0 + cell_m, region[2]), min(y0 + cell_m, region[3])
            c0 = min(max(int(math.floor((x0 - ox) / res + 1e-9)), 0), w)
            c1 = min(max(int(math.ceil((x1 - ox) / res - 1e-9)), 0), w)
            r0 = min(max(int(math.floor((y0 - oy) / res + 1e-9)), 0), h)
            r1 = min(max(int(math.ceil((y1 - oy) / res - 1e-9)), 0), h)
            total = (c1 - c0) * (r1 - r0)
            if total <= 0:
                continue
            n_unk = integ[r1, c1] - integ[r0, c1] - integ[r1, c0] + integ[r0, c0]
            if n_unk / total > unseen_frac:
                out.append((i, j))
    return out


class CellBook:
    """Учёт крупных клеток: облетённые, проваленные, число неудач (retries повторов после ABORTED)."""

    def __init__(self, max_retries=2):
        self.max_retries = int(max_retries)
        self.visited = set()
        self.failed = set()
        self.aborts = {}

    def skip(self):
        return self.visited | self.failed

    def on_success(self, key):
        self.visited.add(key)

    def on_abort(self, key):
        """Неудача: True = клетку ещё можно пробовать, False = клетка теперь проваленная."""
        n = self.aborts.get(key, 0) + 1
        self.aborts[key] = n
        if n > self.max_retries:
            self.failed.add(key)
            return False
        return True


def rank_cells(keys, region, cell_m, pose, skip=(), goal_key=None, hysteresis=1.5,
               others=(), assign_radius=4.0, other_penalty=6.0):
    """Кандидаты [(цена, x, y, key)], по возрастанию цены.

    keys: непросмотренные клетки; skip: облетённые и проваленные; goal_key: клетка текущей цели
    (ей бонус hysteresis); others: цели других дронов [(x, y)]: штраф, если центр ближе assign_radius.
    """
    out = []
    skip = set(skip)
    for key in keys:
        if key in skip:
            continue
        x, y = cell_center(key, region, cell_m)
        c = math.hypot(x - pose[0], y - pose[1])
        if key == goal_key:
            c -= hysteresis
        if any(math.hypot(x - ox, y - oy) < assign_radius for ox, oy in others):
            c += other_penalty
        out.append((float(c), x, y, key))
    out.sort(key=lambda e: (e[0], e[3]))
    return out


class DoneTracker:
    """Конец исследования: кандидатов нет done_cycles циклов подряд (и нет текущей цели)."""

    def __init__(self, done_cycles=3):
        self.done_cycles = int(done_cycles)
        self.empty = 0

    def update(self, n_candidates, has_goal=False):
        if n_candidates == 0 and not has_goal:
            self.empty += 1
        else:
            self.empty = 0
        return self.empty >= self.done_cycles


def goal_timeout(dist, speed, margin=10.0, floor=20.0):
    """Тайм-аут цели, с: двойное время полёта по прямой плюс запас."""
    return max(floor, 2.0 * dist / max(speed, 0.1) + margin)

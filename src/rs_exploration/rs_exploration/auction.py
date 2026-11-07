"""Децентрализованный аукцион: одно и то же жадное распределение на общей таблице ставок.

Без центрального узла: каждый робот держит у себя последние ставки всех и запускает
assign(). Результат детерминирован (порядок: цена, id робота, координаты), поэтому при
одинаковых таблицах все роботы получают одно и то же распределение.
"""
import math


def assign(bids, radius):
    """bids: {robot_id: [(cost, x, y), ...]}; возвращает {robot_id: (x, y, cost)}.

    Робот получает лучшую из своих целей, если у него ещё ничего нет и ни одна уже
    назначенная цель не ближе radius метров.
    """
    entries = sorted((float(c), rid, float(x), float(y))
                     for rid, lst in bids.items() for (c, x, y) in lst)
    got = {}
    taken = []
    for c, rid, x, y in entries:
        if rid in got:
            continue
        if any(math.hypot(x - tx, y - ty) < radius for tx, ty in taken):
            continue
        got[rid] = (x, y, c)
        taken.append((x, y))
    return got

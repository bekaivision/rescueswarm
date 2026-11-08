"""Децентрализованный аукцион: одно и то же жадное распределение на общей таблице ставок.

Без центрального узла: каждый робот держит у себя последние ставки всех и запускает
assign(). Результат детерминирован (порядок: цена, id робота, координаты), поэтому при
одинаковых таблицах все роботы получают одно и то же распределение.

Ставка = (цена, x, y) для фронтира или (цена, x, y, ключ) для осмотра кандидата-человека.
Осмотр: не больше одного робота на ключ (разные роботы могут ставить на разные точки одного
кандидата); радиус между целями для осмотров не проверяется. Приоритет осмотра над фронтирами
задаёт сама цена (вызывающий вычитает большой бонус).
"""
import math


def assign(bids, radius):
    """bids: {robot_id: [(cost, x, y[, key]), ...]}; возвращает {robot_id: (x, y, cost)}.

    Робот получает лучшую из своих целей, если у него ещё ничего нет и ни одна уже
    назначенная цель фронтира не ближе radius метров (для осмотра: ключ не занят).
    """
    entries = []
    for rid, lst in bids.items():
        for b in lst:
            key = b[3] if len(b) > 3 else ''
            entries.append((float(b[0]), rid, float(b[1]), float(b[2]), key or ''))
    entries.sort()
    got = {}
    taken = []
    keys = set()
    for c, rid, x, y, key in entries:
        if rid in got:
            continue
        if key:
            if key in keys:
                continue
            keys.add(key)
        elif any(math.hypot(x - tx, y - ty) < radius for tx, ty in taken):
            continue
        got[rid] = (x, y, c)
        if not key:
            taken.append((x, y))
    return got

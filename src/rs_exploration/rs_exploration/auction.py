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


def make_key(mid, x, y):
    """Глобальный ключ осмотра: локальный id метки + позиция кандидата (id у разных роботов могут не совпадать)."""
    return f'{mid}@{x:.2f},{y:.2f}'


def key_pos(key):
    """(x, y) из глобального ключа или None, если ключ без позиции."""
    if '@' not in key:
        return None
    try:
        xs, ys = key.split('@', 1)[1].split(',')
        return float(xs), float(ys)
    except ValueError:
        return None


def key_local_id(key):
    """Локальный id метки из глобального ключа (для обычного ключа возвращается он сам)."""
    return key.split('@', 1)[0]


def need_home(done, has_goal, homing, reached, fails, retries, dist, tol):
    """Пора ли отправлять робота на базу: всё исследовано, цели нет, ещё не дома и попытки не кончились."""
    return bool(done and not has_goal and not homing and not reached and fails < retries and dist > tol)


def assign(bids, radius, key_radius=0.0):
    """bids: {robot_id: [(cost, x, y[, key]), ...]}; возвращает {robot_id: (x, y, cost)}.

    Робот получает лучшую из своих целей, если у него ещё ничего нет и ни одна уже
    назначенная цель фронтира не ближе radius метров (для осмотра: ключ не занят).
    key_radius > 0: ключи с позицией (см. make_key) ближе key_radius тоже считаются одним кандидатом.
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
    key_xy = []
    for c, rid, x, y, key in entries:
        if rid in got:
            continue
        if key:
            if key in keys:
                continue
            kp = key_pos(key) if key_radius > 0 else None
            if kp is not None and any(math.hypot(kp[0] - qx, kp[1] - qy) < key_radius for qx, qy in key_xy):
                continue
            keys.add(key)
            if kp is not None:
                key_xy.append(kp)
        elif any(math.hypot(x - tx, y - ty) < radius for tx, ty in taken):
            continue
        got[rid] = (x, y, c)
        if not key:
            taken.append((x, y))
    return got

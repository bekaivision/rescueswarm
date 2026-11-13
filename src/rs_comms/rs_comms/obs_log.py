"""Журнал пачек наблюдений по источникам (чистый python, без ROS).

Для каждого источника хранит seq -> сериализованная ObsBatch (байты дёшево отправлять дальше).
have(src) = наибольшее k, при котором есть все пачки 1..k. Пачки отдаются получателю по порядку, по одному разу.
"""


class ObsLog:
    def __init__(self):
        self._data = {}         # источник -> {seq: bytes}
        self._have = {}         # источник -> непрерывный префикс
        self.next_deliver = {}  # источник -> какую seq отдать следующей

    def sources(self):
        return list(self._data)

    def have(self, src):
        return self._have.get(src, 0)

    def add(self, src, seq, data):
        """Добавить пачку. Возвращает [(seq, data)], ставшие доступными по порядку (каждая ровно один раз)."""
        d = self._data.setdefault(src, {})
        self._have.setdefault(src, 0)
        nd = self.next_deliver.setdefault(src, 1)
        seq = int(seq)
        if seq < 1 or seq in d:
            return []
        d[seq] = bytes(data)
        h = self._have[src]
        while (h + 1) in d:
            h += 1
        self._have[src] = h
        out = []
        while nd in d:
            out.append((nd, d[nd]))
            nd += 1
        self.next_deliver[src] = nd
        return out

    def get_range(self, src, a, b):
        """Пачки с a < seq <= b, которые у нас есть, по возрастанию seq."""
        d = self._data.get(src, {})
        return [(k, d[k]) for k in range(int(a) + 1, int(b) + 1) if k in d]

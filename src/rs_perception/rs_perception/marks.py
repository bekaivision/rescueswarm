"""Логика слияния отметок (без ROS): Mark, Fusion. Пороги задаются словарём params."""
import math
import statistics

CANDIDATE, CONFIRMED, REJECTED = 0, 1, 2
NAMES = {CANDIDATE: 'CANDIDATE', CONFIRMED: 'CONFIRMED', REJECTED: 'REJECTED'}

DEFAULTS = dict(
    gate_m=1.0, close_m=3.0, min_close_obs=3,
    temp_lo=305.0, temp_hi=313.0,
    max_height_m=0.4, len_lo=1.1, len_hi=2.2,   # len_* только для отчёта и уверенности
    reject_len_m=0.25,        # меньше этой длины (максимум по близким видам) - явно не человек
    reject_len_obs=3,         # ...при стольких замерах длины
    weight_r0=0.5,
    # частичные виды лежащего тела (1.7 м) дают отметки на его концах дальше gate_m: два "человеческих"
    # (температура в полосе, не слишком высокие, температуры близки) в пределах body_m - одно тело.
    # Поколения мира держат людей на расстоянии >= 2.4 м конец в конец, ловушки >= 2 м (generate.py).
    body_merge_m=1.8, body_dtemp_k=1.0,
    hot_reject_k=320.0,       # слишком горячо: отвергаем по >= hot_reject_obs замерам любой дальности
    hot_reject_obs=3,
    min_obs_report=2,         # отметки с меньшим числом замеров не публикуются (одиночные выбросы)
    # отрицательное свидетельство: кандидат в поле зрения вблизи, а детекций нет -> отвергаем
    neg_range_m=3.0,          # ближе этого расстояния
    neg_min_range_m=0.4,
    neg_half_fov=0.40,        # рад от оси камеры (hfov 60 градусов = +-0.52, берём с запасом)
    neg_time_s=2.0,           # суммарное время в поле зрения без детекций
    neg_gap_s=0.6,            # разрыв больше этого обнуляет накопление
    neg_cam_forward_m=0.25,   # смещение камеры вперёд от центра робота
    # RGB (YOLO) - дополнительный признак: не создаёт отметок и ничего не отвергает
    rgb_range_m=5.0,          # дальше этого RGB-рамки к отметкам не привязываем
    rgb_tol_rad=0.05,         # допуск по углу сверх половины ширины рамки
    rgb_confirm_hits=3,       # столько RGB-попаданий + температура = CONFIRMED (если высота неизвестна)
    rgb_conf_bonus=0.2,       # добавка к уверенности CANDIDATE при полном числе попаданий
    # Фаза 4 (SLAM): поза ошибается на 0.2-0.5 м, и детекции уже отвергнутого объекта уходят дальше gate_m.
    # Тогда рядом рождалась новая отметка и подтверждалась по нескольким неверным замерам высоты.
    # 1) Детекция в пределах shadow_m от отметки, отвергнутой по свойствам объекта (SHADOW_REASONS),
    #    приписывается ей и не создаёт новую отметку. Люди стоят от ловушек не ближе 2 м центр к центру
    #    (generate.py), поэтому 1.3 м не задевает центр тела; детекции конца тела уйдут в ловушку,
    #    но середина тела даёт свою отметку.
    # 2) CONFIRMED по высоте только после min_height_obs замеров высоты (раньше хватало одного).
    #    Больше 3 нельзя: у части людей высота известна лишь в 2-3 близких видах (seed 2 slam).
    shadow_m=1.3,
    min_height_obs=3,
)

# отвергнута по свойствам самого объекта (не "не увидели вблизи"): её соседи - тот же объект
SHADOW_REASONS = ('too_hot', 'too_tall', 'temp_out_of_band')


def p_bonus(p, m):
    return p['rgb_conf_bonus'] * min(1.0, m.rgb_hits / max(p['rgb_confirm_hits'], 1))


class Mark:
    def __init__(self, mid, t):
        self.id = mid
        self.status = CANDIDATE
        self.sx = self.sy = self.sw = 0.0
        self.n_obs = 0
        self.n_air = 0      # сколько замеров от дронов (Фаза 5)
        self.robots = []
        self.min_range = float('inf')
        self.temps_close, self.temps_all = [], []
        self.heights, self.lengths = [], []
        self.n_close = 0
        self.first_seen = t
        self.status_time = t
        self.history = [(t, 'CANDIDATE', 'first_detection')]
        self.reason = 'first_detection'
        self.neg = {}       # робот -> [накоплено с, время последнего кадра]
        self.len_hi = DEFAULTS['len_hi']
        self.rgb_hits = 0
        self.rgb_score_max = 0.0
        self.rgb_robots = []

    @property
    def x(self):
        return self.sx / self.sw

    @property
    def y(self):
        return self.sy / self.sw

    def mean_temp(self):
        # медиана: редкие замеры огня рядом (599 K) не должны выводить человека из полосы температур (Фаза 4)
        v = self.temps_close or self.temps_all
        return statistics.median(v) if v else float('nan')

    def height(self):
        return statistics.median(self.heights) if self.heights else float('nan')

    def length(self):
        # лучший вид: слишком длинные замеры (слились со стеной) не считаем
        if not self.lengths:
            return float('nan')
        ok = [v for v in self.lengths if v <= self.len_hi]
        return max(ok) if ok else max(self.lengths)


class Fusion:
    def __init__(self, params=None, air_robots=()):
        # air_robots: дроны (Фаза 5). Их детекции создают и обновляют CANDIDATE и идут в too_hot,
        # но не считаются близкими, не дают высоту/длину и не подтверждают человека.
        self.air_robots = set(air_robots or ())
        self.p = dict(DEFAULTS)
        if params:
            self.p.update(params)
        self.marks = []
        self.next_id = 1
        self.removed = []    # id слитых отметок (для удаления маркеров)
        self.rgb_unmatched = 0   # RGB-рамки без отметки (только для отчёта)
        self.rgb_total = 0

    def _set(self, m, status, t, reason):
        m.status = status
        m.status_time = t
        m.reason = reason
        m.history.append((t, NAMES[status], reason))

    def update(self, x, y, robot, rng, temp, height, length, t):
        """Одна детекция. Возвращает (Mark, сменился_ли_статус)."""
        p = self.p
        best, bd = None, p['gate_m']
        for m in self.marks:
            d = math.hypot(m.x - x, m.y - y)
            if d <= bd:
                best, bd = m, d
        if best is None:
            best = self._shadow(x, y)
        if best is None:
            best = Mark(f'mark_{self.next_id}', t)
            best.len_hi = p['len_hi']
            self.next_id += 1
            self.marks.append(best)
        m = best
        w = 1.0 / (rng + p['weight_r0']) ** 2
        m.sx += w * x
        m.sy += w * y
        m.sw += w
        m.n_obs += 1
        air = robot in self.air_robots
        if air:
            m.n_air += 1
        else:
            m.neg = {}   # есть детекция: накопленное "не видим" обнуляем (дрон отрицательное свидетельство не трогает)
        if robot not in m.robots:
            m.robots.append(robot)
        m.min_range = min(m.min_range, rng)
        m.temps_all.append(temp)
        close = rng <= p['close_m'] and not air
        if close:
            m.n_close += 1
            m.temps_close.append(temp)
            if not math.isnan(height):
                m.heights.append(height)
            if not math.isnan(length):
                m.lengths.append(length)
        changed = self._evaluate(m, t)
        merged = self._merge(m)
        if merged is not m:
            changed = self._evaluate(merged, t) or changed
        return merged, changed

    def _shadow(self, x, y):
        """Ближайшая отметка, отвергнутая по свойствам объекта, в пределах shadow_m (или None)."""
        best, bd = None, self.p['shadow_m']
        for m in self.marks:
            if m.status != REJECTED or m.reason.split('+')[0] not in SHADOW_REASONS:
                continue
            d = math.hypot(m.x - x, m.y - y)
            if d <= bd:
                best, bd = m, d
        return best

    def _person_like(self, m):
        p = self.p
        if m.status == REJECTED:
            return False
        t = m.mean_temp()
        if math.isnan(t) or not (p['temp_lo'] <= t <= p['temp_hi']):
            return False
        h = m.height()
        return math.isnan(h) or h <= p['max_height_m']

    def _same_object(self, a, b):
        d = math.hypot(a.x - b.x, a.y - b.y)
        if d <= self.p['gate_m']:
            return True
        if d > self.p['body_merge_m']:
            return False
        if not (self._person_like(a) and self._person_like(b)):
            return False
        # отметка только из воздуха: температура верхней грани с дрона не сравнима с температурой
        # наземного вида (другое разрешение и ядро пятна), сравнение температур пропускаем
        if a.n_air == a.n_obs or b.n_air == b.n_obs:
            return True
        return abs(a.mean_temp() - b.mean_temp()) <= self.p['body_dtemp_k']

    def _merge(self, m):
        """Две отметки одного объекта (разные виды одного тела): сливаем в старую."""
        while True:
            other = None
            for o in self.marks:
                if o is not m and self._same_object(o, m):
                    other = o
                    break
            if other is None:
                return m
            a, b = (m, other) if self.marks.index(m) < self.marks.index(other) else (other, m)
            keep_status = a.status if a.n_close >= b.n_close else b.status
            keep_reason = a.reason if a.n_close >= b.n_close else b.reason
            if CONFIRMED in (a.status, b.status) and REJECTED not in (a.status, b.status):
                keep_status, keep_reason = CONFIRMED, (a.reason if a.status == CONFIRMED else b.reason)
            a.sx += b.sx
            a.sy += b.sy
            a.sw += b.sw
            a.n_obs += b.n_obs
            a.n_air += b.n_air
            a.n_close += b.n_close
            a.robots += [r for r in b.robots if r not in a.robots]
            a.min_range = min(a.min_range, b.min_range)
            a.temps_close += b.temps_close
            a.temps_all += b.temps_all
            a.heights += b.heights
            a.lengths += b.lengths
            a.first_seen = min(a.first_seen, b.first_seen)
            a.rgb_hits += b.rgb_hits
            a.rgb_score_max = max(a.rgb_score_max, b.rgb_score_max)
            a.rgb_robots += [r for r in b.rgb_robots if r not in a.rgb_robots]
            a.neg = {}
            if keep_status != a.status:
                self._set(a, keep_status, max(a.status_time, b.status_time), keep_reason + '+merge')
            self.marks.remove(b)
            self.removed.append(b.id)
            m = a

    def _evaluate(self, m, t):
        p = self.p
        # медиана, не среднее: одиночное пятно огня, попавшее на отметку человека (проекция на лидар
        # человека за огнём), не должно отвергать человека (seed 2 slam, Фаза 4)
        if (m.status == CANDIDATE and m.n_obs >= p['hot_reject_obs']
                and statistics.median(m.temps_all) > p['hot_reject_k']):
            self._set(m, REJECTED, t, 'too_hot')
            return True
        if m.status != CANDIDATE or m.n_close < p['min_close_obs']:
            return False
        temp = m.mean_temp()
        temp_ok = p['temp_lo'] <= temp <= p['temp_hi']
        h, ln = m.height(), m.length()
        h_known, l_known = not math.isnan(h), not math.isnan(ln)
        too_tall = h_known and h > p['max_height_m']
        # Длина по лидару ненадёжна (вид с торца, частичный вид), поэтому в решение не входит:
        # ранние короткие замеры отвергали людей. Решают температура и высота (замер лидара).
        if not temp_ok:
            new, why = REJECTED, 'temp_out_of_band'
        elif too_tall:
            new, why = REJECTED, 'too_tall'
        elif h_known and h <= p['max_height_m']:
            if len(m.heights) < p['min_height_obs']:
                return False    # мало замеров высоты: ждём ещё (ошибка позы даёт выбросы высоты)
            new, why = CONFIRMED, 'person_shape_and_temp'
        elif not h_known and m.rgb_hits >= p['rgb_confirm_hits']:
            new, why = CONFIRMED, 'temp_and_rgb'    # высоты нет, но температура и RGB согласны
        else:
            return False    # форма пока не подходит и не отвергает: ждём ещё видов
        self._set(m, new, t, why)
        return True

    def confidence(self, m):
        if m.status == REJECTED:
            return 0.05
        s = 1.0 - math.exp(-(0.15 * m.n_obs + 0.5 * m.n_close) / 2.0)
        temp = m.mean_temp()
        fit = max(0.0, 1.0 - abs(temp - 309.0) / 10.0) if not math.isnan(temp) else 0.0
        c = s * (0.4 + 0.6 * fit)
        if m.status == CANDIDATE:
            c += p_bonus(self.p, m)
            return min(c, 0.99)
        floor = 0.9 if m.rgb_hits >= 1 else 0.8
        return min(max(c, floor), 1.0)

    def reported(self):
        return [m for m in self.marks if m.n_obs >= self.p['min_obs_report']]

    def needs_inspection(self):
        """Открытые кандидаты: ещё не решены (мало близких видов или форма пока неоднозначна)."""
        return [m for m in self.reported() if m.status == CANDIDATE]

    def observe_view(self, robot, rx, ry, yaw, t, los=None):
        """Поза робота: кандидаты в поле зрения камеры вблизи без детекций копят время, потом REJECTED.

        los(x0, y0, x1, y1) -> bool: прямая видимость на карте (None = не проверять).
        Возвращает список отвергнутых отметок.
        """
        p = self.p
        cx = rx + p['neg_cam_forward_m'] * math.cos(yaw)
        cy = ry + p['neg_cam_forward_m'] * math.sin(yaw)
        out = []
        for m in self.marks:
            if m.status != CANDIDATE or m.n_obs < p['min_obs_report']:
                continue
            d = math.hypot(m.x - cx, m.y - cy)
            ang = (math.atan2(m.y - cy, m.x - cx) - yaw + math.pi) % (2 * math.pi) - math.pi
            if not (p['neg_min_range_m'] <= d <= p['neg_range_m'] and abs(ang) <= p['neg_half_fov']):
                m.neg.pop(robot, None)
                continue
            if los is not None and not los(cx, cy, m.x, m.y):
                m.neg.pop(robot, None)
                continue
            acc, last = m.neg.get(robot, (0.0, t))
            if t - last > p['neg_gap_s']:
                acc = 0.0
            else:
                acc += max(t - last, 0.0)
            m.neg[robot] = (acc, t)
            if acc >= p['neg_time_s']:
                self._set(m, REJECTED, t, 'not_seen_close')
                out.append(m)
        return out

    def rgb_observe(self, robot, rx, ry, yaw, bearing, half_w, score, t, los=None):
        """RGB-рамка человека (угол от оси камеры, + влево). Привязка к отметке по углу; возвращает Mark или None.

        Только добавляет подтверждение: отметок не создаёт и не отвергает. Увиденная по RGB отметка
        не может быть отвергнута как not_seen_close (сбрасываем накопление).
        """
        p = self.p
        self.rgb_total += 1
        cx = rx + p['neg_cam_forward_m'] * math.cos(yaw)
        cy = ry + p['neg_cam_forward_m'] * math.sin(yaw)
        tol = half_w + p['rgb_tol_rad']
        cands = []
        for m in self.reported():
            if m.status == REJECTED:
                continue
            d = math.hypot(m.x - cx, m.y - cy)
            if not (p['neg_min_range_m'] <= d <= p['rgb_range_m']):
                continue
            ang = (math.atan2(m.y - cy, m.x - cx) - yaw + math.pi) % (2 * math.pi) - math.pi
            off = abs(ang - bearing)
            if off > tol:
                continue
            if los is not None and not los(cx, cy, m.x, m.y):
                continue
            cands.append((d, off, m))
        if not cands:
            self.rgb_unmatched += 1
            return None
        # несколько отметок в рамке: ближайшая по дальности; иначе по углу
        m = min(cands, key=lambda c: (c[0], c[1]))[2] if len(cands) > 1 else cands[0][2]
        m.rgb_hits += 1
        m.rgb_score_max = max(m.rgb_score_max, float(score))
        if robot not in m.rgb_robots:
            m.rgb_robots.append(robot)
        m.neg = {}
        self._evaluate(m, t)
        return m

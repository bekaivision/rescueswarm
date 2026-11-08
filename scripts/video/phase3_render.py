#!/usr/bin/env python3
"""Сборка видео Фазы 3 из записи прогона (внутри контейнера, без окон).

  phase3_render.py RAW_DIR LOGO.png OUT.mp4
RAW_DIR: rec.npz (phase3_rec.py), run/{coverage.csv,gt.json,check.json}, seed.txt. Кадры рисуются на PIL и идут в ffmpeg.
Карта, пути, цели, отметки людей и тепловые кадры берутся из записи; покрытие из coverage.csv; счётчики
считаются по отметкам и эталону (gt.json) на момент времени, итог в начале и в конце совпадает с check.json.
Раскладка: 0-3 с итог (последний кадр записи), затем таймлапс x F, затем логотип.
"""
import csv
import json
import math
import os
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from phase2_render import (BG, C_FREE, C_OCC, C_UNKNOWN, F, FPS, H, ROBOT_COLORS, TEAL, W,  # noqa: E402
                           boxed, centered, font)

MAP_X, MAP_Y, MAP_W = 20, 140, 780
SS = 2
RX, RW = 818, 242                                   # правая колонка
INTRO_S, END_HOLD_S, OUTRO_S, TL_MAX_S = 3.0, 1.0, 3.5, 21.0
HOT_K = 303.0
HOLD_S = 8.0                                        # сим. секунд: сколько держим кадр с пятном
CAND, CONF, REJ = (255, 205, 60), (60, 230, 100), (150, 156, 164)
STATUS_COL = {0: CAND, 1: CONF, 2: REJ}
MATCH_M = 1.0


def match(pts, objs):
    """Жадно, один к одному, порог MATCH_M. pts, objs: списки (x, y). Возвращает множество индексов objs."""
    pairs = sorted((math.hypot(p[0] - o[0], p[1] - o[1]), i, j)
                   for i, p in enumerate(pts) for j, o in enumerate(objs))
    ui, uj, out = set(), set(), set()
    for d, i, j in pairs:
        if d > MATCH_M:
            break
        if i in ui or j in uj:
            continue
        ui.add(i)
        uj.add(j)
        out.add(j)
    return out


def lut():
    pts = [(0.0, (0, 0, 8)), (0.25, (60, 10, 100)), (0.5, (190, 50, 70)), (0.75, (250, 150, 30)), (1.0, (255, 255, 210))]
    xs = np.linspace(0, 1, 256)
    return np.stack([np.interp(xs, [p[0] for p in pts], [p[1][c] for p in pts]) for c in range(3)], axis=1).astype(np.uint8)


LUT = lut()


class Data:
    def __init__(self, raw):
        z = np.load(os.path.join(raw, 'rec.npz'))
        self.maps, self.map_t = z['maps'], z['map_t']
        self.res, self.ox, self.oy = float(z['meta'][0]), float(z['meta'][1]), float(z['meta'][2])
        self.pose, self.goal, self.marks = z['pose'], z['goal'], z['marks']
        self.th, self.th_t, self.th_r = z['th'], z['th_t'], z['th_r']
        run = os.path.join(raw, 'run')
        with open(os.path.join(run, 'coverage.csv')) as f:
            rows = list(csv.DictReader(f))
        self.cov_t = np.array([float(r['sim_time_s']) for r in rows])
        self.cov = 100 * np.array([float(r['coverage']) for r in rows])
        gt = json.load(open(os.path.join(run, 'gt.json')))
        self.people = [p for p in gt['people']]
        self.reach = [p for p in self.people if p.get('reachable', True)]
        self.decoys = gt['decoys']
        self.check = json.load(open(os.path.join(run, 'check.json')))
        self.seed = open(os.path.join(raw, 'seed.txt')).read().strip() if os.path.exists(os.path.join(raw, 'seed.txt')) else '?'
        self.n = int(self.pose[:, 1].max()) + 1
        self.tr = {}
        for i in range(self.n):
            p = self.pose[self.pose[:, 1] == i]
            self.tr[i] = (p[:, 0], p[:, 2], p[:, 3], p[:, 4])
        self.t0 = float(self.cov_t[0])
        self.t_end = float(min(self.cov_t[-1], self.map_t[-1]))
        self.map_h = MAP_W * self.maps.shape[1] / self.maps.shape[2]
        self.mark_t = np.unique(self.marks[:, 0]) if len(self.marks) else np.zeros(0)
        self._cache = {}
        self._build_thermal_choice()

    # --- тепловой кадр: берём самый свежий кадр с горячим пятном (не старше HOLD_S, иначе показываем свежий кадр)
    def _build_thermal_choice(self):
        self.th_hot = (self.th.astype(np.float32) * 0.01 > HOT_K).reshape(len(self.th), -1).sum(axis=1)
        self.hot_idx = np.nonzero(self.th_hot >= 4)[0]
        self.hot_t = self.th_t[self.hot_idx]

    def thermal_at(self, t):
        k = int(np.searchsorted(self.hot_t, t, side='right') - 1)
        if k >= 0 and t - self.hot_t[k] <= HOLD_S:
            i = int(self.hot_idx[k])
        else:
            idx = np.nonzero(self.th_t <= t)[0]
            if not len(idx):
                return 0, None
            i = int(idx[-1])
            if k >= 0:                      # робот последнего горячего кадра, чтобы вид не прыгал
                j = np.nonzero((self.th_r == self.th_r[int(self.hot_idx[k])]) & (self.th_t <= t))[0]
                i = int(j[-1])
        return int(self.th_r[i]), self.th[i].astype(np.float32) * 0.01

    def coverage(self, t):
        return float(np.interp(t, self.cov_t, self.cov))

    def map_rgb(self, k):
        if k not in self._cache:
            if len(self._cache) > 4:
                self._cache.clear()
            m = self.maps[k]
            img = np.empty(m.shape + (3,), np.uint8)
            img[:] = C_FREE
            img[m < 0] = C_UNKNOWN
            img[m >= 50] = C_OCC
            self._cache[k] = img[::-1]
        return self._cache[k]

    def map_at(self, t):
        k = int(np.clip(np.searchsorted(self.map_t, t, side='right') - 1, 0, len(self.map_t) - 1))
        a = self.map_rgb(k)
        if k + 1 < len(self.map_t) and t > self.map_t[k]:
            w = (t - self.map_t[k]) / (self.map_t[k + 1] - self.map_t[k])
            return (a * (1 - w) + self.map_rgb(k + 1) * w).astype(np.uint8)
        return a

    def to_px(self, x, y):
        s = SS * MAP_W / (self.maps.shape[2] * self.res)
        return (np.asarray(x) - self.ox) * s, (self.maps.shape[1] * self.res - (np.asarray(y) - self.oy)) * s

    def goal_of(self, i, t):
        g = self.goal[(self.goal[:, 1] == i) & (self.goal[:, 0] <= t)]
        if len(g) and g[-1, 2] > 0.5 and t - g[-1, 0] < 4:
            return g[-1, 3], g[-1, 4]
        return None

    def marks_at(self, t):
        """Список (id, status, x, y, status_time) последнего сообщения /people/marks не позже t."""
        if not len(self.mark_t):
            return []
        k = int(np.searchsorted(self.mark_t, t, side='right') - 1)
        if k < 0:
            return []
        r = self.marks[self.marks[:, 0] == self.mark_t[k]]
        return [(int(a[1]), int(a[2]), a[3], a[4], a[5]) for a in r]

    def counters(self, t):
        ms = self.marks_at(t)
        conf = [(m[2], m[3]) for m in ms if m[1] == 1]
        rej = [(m[2], m[3]) for m in ms if m[1] == 2]
        found = match(conf, [(p['x'], p['y']) for p in self.reach])
        rejd = match(rej, [(d['x'], d['y']) for d in self.decoys])
        return len(found), len(self.reach), len(rejd), len(self.decoys)


def draw_mark(d, cx, cy, status, r):
    col = STATUS_COL[status]
    d.ellipse([cx - r - 4, cy - r - 4, cx + r + 4, cy + r + 4], fill=(255, 255, 255))
    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=col)
    k = r * 0.5
    dark, w = (20, 24, 30), 6
    if status == 1:      # галочка
        d.line([(cx - k, cy), (cx - k * 0.3, cy + k * 0.8), (cx + k, cy - k * 0.8)], fill=(255, 255, 255), width=w + 1, joint='curve')
    elif status == 2:    # крестик
        d.line([(cx - k * 0.8, cy - k * 0.8), (cx + k * 0.8, cy + k * 0.8)], fill=dark, width=w)
        d.line([(cx - k * 0.8, cy + k * 0.8), (cx + k * 0.8, cy - k * 0.8)], fill=dark, width=w)
    else:                # знак вопроса
        f = font(int(r * 1.5))
        d.text((cx, cy), '?', font=f, fill=dark, anchor='mm')


def map_panel(D, t):
    rgb = D.map_at(t)
    mh = int(round(D.map_h))
    im = Image.fromarray(rgb).resize((MAP_W * SS, mh * SS), Image.BILINEAR)
    d = ImageDraw.Draw(im)
    for i in range(D.n):
        col = ROBOT_COLORS[i % 3]
        tt, xs, ys, yaws = D.tr[i]
        if t < tt[0]:
            continue
        j = np.searchsorted(tt, t, side='right')
        px, py = D.to_px(np.append(xs[:j], np.interp(t, tt, xs)), np.append(ys[:j], np.interp(t, tt, ys)))
        if len(px) > 1:
            d.line(list(zip(px, py)), fill=col, width=5, joint='curve')
        cx, cy = px[-1], py[-1]
        g = D.goal_of(i, t)
        if g:
            gx, gy = D.to_px(*g)
            d.polygon([(gx, gy - 14), (gx + 14, gy), (gx, gy + 14), (gx - 14, gy)], outline=col, width=4)
        yaw = float(np.interp(t, tt, np.unwrap(yaws)))
        r = 0.3 * SS * MAP_W / (D.maps.shape[2] * D.res)
        d.ellipse([cx - r - 3, cy - r - 3, cx + r + 3, cy + r + 3], fill=(255, 255, 255))
        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=col)
        d.line([(cx, cy), (cx + 1.6 * r * math.cos(yaw), cy - 1.6 * r * math.sin(yaw))], fill=(255, 255, 255), width=5)
    s = SS * MAP_W / (D.maps.shape[2] * D.res)
    for mid, st, x, y, st_t in D.marks_at(t):
        cx, cy = D.to_px(x, y)
        if not (30 < cx < MAP_W * SS - 30 and 30 < cy < D.map_h * SS - 30):
            continue
        age = t - st_t
        if 0 <= age < 8.0:       # вспышка при смене статуса
            rr = 0.42 * s * (1 + 1.6 * age / 8.0)
            d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], outline=STATUS_COL[st], width=6)
        draw_mark(d, cx, cy, st, 0.42 * s)
    return im.resize((MAP_W, mh), Image.LANCZOS)


def thermal_panel(D, t):
    r, temp = D.thermal_at(t)
    w, h = RW, int(RW * 0.75)
    if temp is None:
        return Image.new('RGB', (w, h), (0, 0, 8)), r
    g = np.clip((temp - 280.0) / 40.0, 0, 1)
    im = Image.fromarray(LUT[(g * 255).astype(np.uint8)]).resize((w, h), Image.BILINEAR)
    d = ImageDraw.Draw(im)
    lab, n = ndimage.label(temp > HOT_K, structure=np.ones((3, 3)))
    sx, sy = w / temp.shape[1], h / temp.shape[0]
    for k, sl in enumerate(ndimage.find_objects(lab), start=1):
        m = lab[sl] == k
        if m.sum() < 4:
            continue
        pk = float(temp[sl][m].max())
        x0, x1, y0, y1 = sl[1].start * sx, sl[1].stop * sx, sl[0].start * sy, sl[0].stop * sy
        d.rectangle([x0 - 2, y0 - 2, x1 + 2, y1 + 2], outline=(70, 235, 255), width=2)
        d.text((x0, max(2, y0 - 22)), f'{pk:.0f} K', font=font(16), fill=(70, 235, 255))
    return im, r


def cov_panel(D, d, t, x0=345, y0=832, w=435, h=70):
    d.line([(x0, y0 + h), (x0 + w, y0 + h)], fill=(90, 100, 112), width=2)
    d.line([(x0, y0), (x0, y0 + h)], fill=(90, 100, 112), width=2)
    for lvl in (50, 95):
        yy = y0 + h - h * lvl / 100
        d.line([(x0, yy), (x0 + w, yy)], fill=(50, 60, 72), width=1)
        d.text((x0 - 8 - d.textlength(f'{lvl}', font=F['s18']), yy - 11), f'{lvl}', font=F['s18'], fill=(130, 140, 150))
    tmax = D.t_end - D.t0
    m = D.cov_t <= t
    xs = x0 + w * (D.cov_t[m] - D.t0) / tmax
    ys = y0 + h - h * D.cov[m] / 100
    pts = list(zip(xs, ys))
    if len(pts) > 1:
        d.polygon(pts + [(xs[-1], y0 + h), (xs[0], y0 + h)], fill=(20, 70, 68))
        d.line(pts, fill=TEAL, width=4)
    cx = x0 + w * (t - D.t0) / tmax
    cy = y0 + h - h * D.coverage(t) / 100
    d.ellipse([cx - 8, cy - 8, cx + 8, cy + 8], fill=(255, 255, 255))
    d.text((x0, y0 + h + 4), 'sim time', font=F['s18'], fill=(130, 140, 150))
    end = f'{tmax:.0f} s'
    d.text((x0 + w - d.textlength(end, font=F['s18']), y0 + h + 4), end, font=F['s18'], fill=(130, 140, 150))


def fit_font(d, text, maxw, sizes=(44, 40, 36, 32, 28)):
    for s in sizes:
        f = font(s)
        if d.textlength(text, font=f) <= maxw:
            return f
    return font(sizes[-1])


def base_frame(D, t, title, caption, speed=None):
    im = Image.new('RGB', (W, H), BG)
    im.paste(map_panel(D, t), (MAP_X, MAP_Y))
    d = ImageDraw.Draw(im)
    boxed(d, (30, 24), 'Simulation', F['s36'])
    if speed:
        w = d.textlength(speed, font=F['s36'])
        d.text((W - 30 - w, 24), speed, font=F['s36'], fill=TEAL)
    d.text(((W - d.textlength(title, font=fit_font(d, title, 1000))) / 2, 84), title, font=fit_font(d, title, 1000), fill=(255, 255, 255))
    cov = D.coverage(t)
    d.text((36, 822), f'{cov:.1f}%', font=F['s70'], fill=TEAL)
    d.text((40, 900), 'map coverage', font=F['s22'], fill=(150, 160, 170))
    cov_panel(D, d, t)
    cf = fit_font(d, caption, 1030, sizes=(30, 27, 24))
    d.text(((W - d.textlength(caption, font=cf)) / 2, 1032), caption, font=cf, fill=(255, 255, 255))
    # правая колонка: тепловизор
    th, r = thermal_panel(D, t)
    d.text((RX, 144), f'THERMAL  ground_{r + 1}', font=font(16), fill=(190, 200, 210))
    im.paste(th, (RX, 168))
    d.rectangle([RX - 2, 166, RX + RW + 1, 168 + th.size[1] + 1], outline=(90, 100, 112), width=2)
    # счётчики (по отметкам и эталону на момент t)
    nf, nr, nd, nt = D.counters(t)
    y = 168 + th.size[1] + 26
    d.text((RX, y), 'REACHABLE PEOPLE FOUND', font=font(16), fill=(190, 200, 210))
    d.text((RX, y + 22), f'{nf}/{nr}', font=F['s84'], fill=CONF)
    y += 140
    d.text((RX, y), 'DECOYS REJECTED', font=font(16), fill=(190, 200, 210))
    d.text((RX, y + 22), f'{nd}/{nt}', font=F['s84'], fill=REJ)
    y += 140
    for st, label in ((0, 'candidate (checking)'), (1, 'confirmed person'), (2, 'rejected (decoy)')):
        draw_mark(d, RX + 14, y + 12, st, 11)
        d.text((RX + 36, y + 2), label, font=font(17, bold=False), fill=(215, 222, 230))
        y += 34
    return im


def outro(logo):
    im = Image.new('RGB', (W, H), BG)
    lg = Image.open(logo).convert('RGBA').resize((640, 640), Image.LANCZOS)
    im.paste(lg, (220, 90), lg)
    d = ImageDraw.Draw(im)
    boxed(d, (30, 24), 'Simulation', F['s36'])
    centered(d, 790, 'Bekai Vision', F['s70'], (255, 255, 255))
    centered(d, 890, 'RescueSwarm  |  Phase 3', F['s44'], TEAL)
    return im


def main():
    raw, logo, out = sys.argv[1:4]
    D = Data(raw)
    T = D.t_end - D.t0
    speed = max(1, math.ceil(T / TL_MAX_S))
    n_tl = int(round(T / speed * FPS))
    n_intro, n_hold, n_out = int(INTRO_S * FPS), int(END_HOLD_S * FPS), int(OUTRO_S * FPS)
    c = D.check
    nf, nr, nd, nt = D.counters(D.t_end)
    print(f'seed {D.seed}: {T:.0f} сим. с, x{speed}, покрытие в конце {D.coverage(D.t_end):.1f} %, '
          f'кадров {n_intro + n_tl + n_hold + n_out}')
    print(f'check.json: найдено достижимых {c["confirmed_reachable"]}/{c["reachable"]}, ловушек отвергнуто '
          f'{c["decoys_rejected"]}/{c["decoys"]}; по записи в конце {nf}/{nr} и {nd}/{nt}')
    types = sorted({d['type'].replace('_', ' ') for d in D.decoys})
    title_res = f'Reachable people found {c["confirmed_reachable"]}/{c["reachable"]}, decoys rejected {c["decoys_rejected"]}/{c["decoys"]}'
    cap_a = 'Robots explore; thermal cameras look for heat sources'
    cap_b = 'Hot spot = candidate: a robot drives closer to check it'
    cap_c = 'Warm body = person; ' + ' / '.join(types) + ' = rejected'
    ff = subprocess.Popen(['ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}',
                           '-r', str(FPS), '-i', '-', '-c:v', 'libx264', '-preset', 'slow', '-crf', '16', '-pix_fmt', 'yuv420p',
                           '-r', str(FPS), '-movflags', '+faststart', '-an', out], stdin=subprocess.PIPE)
    put = lambda im: ff.stdin.write(im.tobytes())     # noqa: E731
    first = base_frame(D, D.t_end, title_res, f'Result after {T:.0f} s of simulation, no human control, seed {D.seed}')
    for _ in range(n_intro):
        put(first)
    ttl = 'Thermal people search, timelapse'
    for k in range(n_tl):
        t = D.t0 + k * speed / FPS
        frac = k / max(1, n_tl - 1)
        cap = cap_a if frac < 0.2 else cap_b if frac < 0.55 else cap_c
        im = base_frame(D, t, ttl, cap, speed=f'x{speed}  t = {t - D.t0:.0f} s')
        if k < 8:
            im = Image.blend(first, im, (k + 1) / 8)
        put(im)
    last = base_frame(D, D.t_end, title_res, cap_c, speed=f'x{speed}  t = {T:.0f} s')
    for _ in range(n_hold):
        put(last)
    ol = outro(logo)
    for k in range(n_out):
        put(Image.blend(last, ol, min(1.0, (k + 1) / 6)) if k < 6 else ol)
    ff.stdin.close()
    ff.wait()
    print('готово', out)


if __name__ == '__main__':
    main()

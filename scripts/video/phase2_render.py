#!/usr/bin/env python3
"""Сборка видео Фазы 2 из записи прогона (внутри контейнера, без окон).

  phase2_render.py RAW_DIR LOGO.png OUT.mp4
RAW_DIR: rec.npz (phase2_rec.py), run/coverage.csv, seed.txt. Кадры рисуются на PIL и идут в ffmpeg.
Числа покрытия берутся из coverage.csv по времени симуляции; карта, пути, цели и фронтиры - из записи.
Раскладка: 0-3 с итог (последний кадр записи), затем таймлапс x F, затем логотип.
"""
import csv
import math
import os
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

W = H = 1080
FPS = 30
BG = (11, 15, 20)
TEAL = (45, 212, 191)
FONTDIR = '/usr/share/fonts/truetype/dejavu/'
ROBOT_COLORS = [(255, 90, 95), (61, 220, 132), (77, 163, 255)]
C_UNKNOWN, C_FREE, C_OCC = (17, 23, 30), (44, 58, 72), (232, 238, 244)
C_FRONT = (255, 205, 60)
MAP_X, MAP_Y, MAP_W, MAP_H = 111, 150, 858, 726      # панель карты: мир 26 x 22 м, 33 px/м
SS = 2                                                # суперсэмплинг панели карты
INTRO_S, END_HOLD_S, OUTRO_S, TL_MAX_S = 3.0, 1.0, 3.5, 20.0


def font(size, bold=True):
    return ImageFont.truetype(FONTDIR + ('DejaVuSans-Bold.ttf' if bold else 'DejaVuSans.ttf'), size)


F = {k: font(v) for k, v in dict(s18=18, s22=22, s30=30, s34=34, s36=36, s44=44, s56=56, s70=70, s84=84).items()}


def centered(d, y, text, f, fill):
    w = d.textlength(text, font=f)
    d.text(((W - w) / 2, y), text, font=f, fill=fill)


def boxed(d, xy, text, f, fill=(255, 255, 255), pad=12, alpha_bg=(0, 0, 0)):
    x, y = xy
    w = d.textlength(text, font=f)
    asc, desc = f.getmetrics()
    d.rectangle([x - pad, y - pad + 4, x + w + pad, y + asc + desc + pad - 4], fill=alpha_bg)
    d.text((x, y), text, font=f, fill=fill)
    return w


class Data:
    def __init__(self, raw):
        z = np.load(os.path.join(raw, 'rec.npz'))
        self.maps, self.map_t = z['maps'], z['map_t']
        self.res, self.ox, self.oy = float(z['meta'][0]), float(z['meta'][1]), float(z['meta'][2])
        self.pose, self.goal = z['pose'], z['goal']
        self.front_t, self.front_cnt, self.front_pts = z['front_t'], z['front_cnt'], z['front_pts']
        self.front_off = np.concatenate([[0], np.cumsum(self.front_cnt)])
        with open(os.path.join(raw, 'run', 'coverage.csv')) as f:
            rows = list(csv.DictReader(f))
        self.cov_t = np.array([float(r['sim_time_s']) for r in rows])
        self.cov = 100 * np.array([float(r['coverage']) for r in rows])
        self.seed = open(os.path.join(raw, 'seed.txt')).read().strip() if os.path.exists(os.path.join(raw, 'seed.txt')) else '?'
        self.n = int(self.pose[:, 1].max()) + 1
        self.tr = {}
        for i in range(self.n):
            p = self.pose[self.pose[:, 1] == i]
            self.tr[i] = (p[:, 0], p[:, 2], p[:, 3], p[:, 4])
        self.t0 = float(self.cov_t[0])
        self.t_end = float(min(self.cov_t[-1], self.map_t[-1]))
        self._cache = {}

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
            self._cache[k] = img[::-1]               # y вверх -> строки сверху вниз
        return self._cache[k]

    def map_at(self, t):
        k = int(np.clip(np.searchsorted(self.map_t, t, side='right') - 1, 0, len(self.map_t) - 1))
        a = self.map_rgb(k)
        if k + 1 < len(self.map_t) and t > self.map_t[k]:
            w = (t - self.map_t[k]) / (self.map_t[k + 1] - self.map_t[k])
            b = self.map_rgb(k + 1)
            return (a * (1 - w) + b * w).astype(np.uint8)
        return a

    def to_px(self, x, y):                            # мир -> пиксели панели (с суперсэмплингом)
        s = SS * MAP_W / (self.maps.shape[2] * self.res)
        return (np.asarray(x) - self.ox) * s, (self.maps.shape[1] * self.res - (np.asarray(y) - self.oy)) * s

    def frontiers(self, t):
        k = int(np.searchsorted(self.front_t, t, side='right') - 1)
        if k < 0 or t - self.front_t[k] > 3:
            return np.zeros((0, 2))
        return self.front_pts[self.front_off[k]:self.front_off[k + 1]]

    def goal_of(self, i, t):
        g = self.goal[(self.goal[:, 1] == i) & (self.goal[:, 0] <= t)]
        if len(g) and g[-1, 2] > 0.5 and t - g[-1, 0] < 4:
            return g[-1, 3], g[-1, 4]
        return None


def map_panel(D, t, trail=True):
    rgb = D.map_at(t)
    im = Image.fromarray(rgb).resize((MAP_W * SS, MAP_H * SS), Image.BILINEAR)
    d = ImageDraw.Draw(im)
    fx, fy = D.to_px(*D.frontiers(t).T) if len(D.frontiers(t)) else ([], [])
    for x, y in zip(fx, fy):
        d.ellipse([x - 2.5, y - 2.5, x + 2.5, y + 2.5], fill=C_FRONT)
    for i in range(D.n):
        col = ROBOT_COLORS[i % 3]
        tt, xs, ys, yaws = D.tr[i]
        if t < tt[0]:
            continue
        j = np.searchsorted(tt, t, side='right')
        px, py = D.to_px(np.append(xs[:j], np.interp(t, tt, xs)), np.append(ys[:j], np.interp(t, tt, ys)))
        if trail and len(px) > 1:
            d.line(list(zip(px, py)), fill=col, width=5, joint='curve')
        cx, cy = px[-1], py[-1]
        g = D.goal_of(i, t)
        if g:
            gx, gy = D.to_px(*g)
            r = 14
            d.polygon([(gx, gy - r), (gx + r, gy), (gx, gy + r), (gx - r, gy)], outline=col, width=4)
        yaw = float(np.interp(t, tt, np.unwrap(yaws)))
        r = 0.3 * SS * MAP_W / (D.maps.shape[2] * D.res)
        d.ellipse([cx - r - 3, cy - r - 3, cx + r + 3, cy + r + 3], fill=(255, 255, 255))
        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=col)
        d.line([(cx, cy), (cx + 1.6 * r * math.cos(yaw), cy - 1.6 * r * math.sin(yaw))], fill=(255, 255, 255), width=5)
    return im.resize((MAP_W, MAP_H), Image.LANCZOS)


def cov_panel(D, d, t, x0=410, y0=898, w=630, h=78):
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


def base_frame(D, t, tag, title, caption, speed=None):
    im = Image.new('RGB', (W, H), BG)
    im.paste(map_panel(D, t), (MAP_X, MAP_Y))
    d = ImageDraw.Draw(im)
    boxed(d, (30, 24), 'Simulation', F['s36'])
    if speed:
        w = d.textlength(speed, font=F['s36'])
        d.text((W - 30 - w, 24), speed, font=F['s36'], fill=TEAL)
    centered(d, 84, title, F['s44'], (255, 255, 255))
    cov = D.coverage(t)
    d.text((36, 884), f'{cov:.1f}%', font=F['s84'], fill=TEAL)
    d.text((40, 975), 'map coverage', font=F['s22'], fill=(150, 160, 170))
    cov_panel(D, d, t)
    centered(d, 1032, caption, F['s30'], (255, 255, 255))
    # условные обозначения в левом нижнем углу карты (там вне здания пусто)
    lx, ly = MAP_X + 8, MAP_Y + MAP_H - 30
    d.ellipse([lx, ly + 6, lx + 10, ly + 16], fill=C_FRONT)
    d.text((lx + 16, ly), 'frontier', font=F['s18'], fill=(190, 200, 210))
    d.polygon([(lx + 100, ly + 11), (lx + 110, ly + 1), (lx + 120, ly + 11), (lx + 110, ly + 21)], outline=(255, 255, 255), width=2)
    d.text((lx + 128, ly), 'robot goal', font=F['s18'], fill=(190, 200, 210))
    return im


def outro(logo):
    im = Image.new('RGB', (W, H), BG)
    lg = Image.open(logo).convert('RGBA').resize((640, 640), Image.LANCZOS)
    im.paste(lg, (220, 90), lg)
    d = ImageDraw.Draw(im)
    boxed(d, (30, 24), 'Simulation', F['s36'])
    centered(d, 790, 'Bekai Vision', F['s70'], (255, 255, 255))
    centered(d, 890, 'RescueSwarm  |  Phase 2', F['s44'], TEAL)
    return im


def main():
    raw, logo, out = sys.argv[1:4]
    D = Data(raw)
    T = D.t_end - D.t0
    speed = max(1, math.ceil(T / TL_MAX_S))
    n_tl = int(round(T / speed * FPS))
    n_intro, n_hold, n_out = int(INTRO_S * FPS), int(END_HOLD_S * FPS), int(OUTRO_S * FPS)
    print(f'seed {D.seed}: {T:.0f} сим. с, x{speed}, покрытие в конце {D.coverage(D.t_end):.1f} %, кадров {n_intro + n_tl + n_hold + n_out}')
    cap_a = 'Empty map: 3 robots start exploring on their own'
    cap_b = 'Robots pick frontiers and split targets between them'
    cap_c = 'One shared map, no human control'
    ff = subprocess.Popen(['ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}',
                           '-r', str(FPS), '-i', '-', '-c:v', 'libx264', '-preset', 'slow', '-crf', '16', '-pix_fmt', 'yuv420p',
                           '-r', str(FPS), '-movflags', '+faststart', '-an', out], stdin=subprocess.PIPE)
    put = lambda im: ff.stdin.write(im.tobytes())     # noqa: E731
    # 1) итог в первые 3 с
    res_cap = f'Result: world mapped by 3 robots, no human, seed {D.seed}'
    first = base_frame(D, D.t_end, 'result', f'{D.coverage(D.t_end):.0f}% explored in {T:.0f} s of simulation', res_cap, speed=None)
    for _ in range(n_intro):
        put(first)
    # 2) таймлапс (первые 8 кадров - перекрёстное затухание из итога в пустую карту)
    for k in range(n_tl):
        t = D.t0 + k * speed / FPS
        frac = k / max(1, n_tl - 1)
        cap = cap_a if frac < 0.25 else cap_b if frac < 0.65 else cap_c
        im = base_frame(D, t, 'tl', 'Autonomous exploration, timelapse', cap, speed=f'x{speed}  t = {t - D.t0:.0f} s')
        if k < 8:
            im = Image.blend(first, im, (k + 1) / 8)
        put(im)
    last = base_frame(D, D.t_end, 'tl', 'Autonomous exploration, timelapse', cap_c, speed=f'x{speed}  t = {T:.0f} s')
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

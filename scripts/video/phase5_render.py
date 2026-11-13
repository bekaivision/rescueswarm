#!/usr/bin/env python3
"""Сборка видео Фазы 5 из записи прогона смешанного роя (внутри контейнера, без окон).

  phase5_render.py RAW_DIR LOGO.png OUT.mp4
RAW_DIR: rec.npz (phase5_rec.py), run/{coverage.csv,gt.json,check.json}, seed.txt, backend.txt.
Общее (карта, отметки, график, тепловой кадр) из phase3_render. Новое: клетки, которые известны только
сверху (от дронов), подкрашены синим; дроны рисуются крестом с винтами и своим путём; справа тепловой
кадр дрона (вид сверху) и покрытие слоёв: сверху и с земли.
"""
import csv
import math
import os
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import phase3_render as P3  # noqa: E402
from phase2_render import (BG, C_FREE, C_OCC, C_UNKNOWN, FPS, H, ROBOT_COLORS, TEAL, W, F, boxed, centered, font)  # noqa: E402
from phase3_render import CONF, REJ, RX, RW, MAP_X, MAP_Y, MAP_W, SS, draw_mark  # noqa: E402

DRONE_COLORS = [(255, 170, 0), (200, 90, 255)]
AIR_FREE = (150, 185, 235)       # свободно, известно только сверху
AIR_OCC = (40, 70, 140)          # занято, известно только сверху


class Data(P3.Data):
    def __init__(self, raw):
        super().__init__(raw)
        z = np.load(os.path.join(raw, 'rec.npz'))
        self.maps_g = z['maps_g']
        dp = z['dpose']
        self.m = int(dp[:, 1].max()) + 1 if len(dp) else 0
        self.dtr = {i: (dp[dp[:, 1] == i][:, 0], dp[dp[:, 1] == i][:, 2], dp[dp[:, 1] == i][:, 3],
                        dp[dp[:, 1] == i][:, 4]) for i in range(self.m)}
        self.dgoal = z['dgoal']
        rows = list(csv.DictReader(open(os.path.join(raw, 'run', 'coverage.csv'))))
        self.cov_air = 100 * np.array([float(r.get('cov_air') or 0) for r in rows])
        self.cov_ground = 100 * np.array([float(r.get('cov_ground') or 0) for r in rows])
        bp = os.path.join(raw, 'backend.txt')
        self.backend = open(bp).read().strip() if os.path.exists(bp) else 'light'

    def map_rgb(self, k):
        if k not in self._cache:
            if len(self._cache) > 4:
                self._cache.clear()
            m, g = self.maps[k], self.maps_g[k]
            img = np.empty(m.shape + (3,), np.uint8)
            img[:] = C_FREE
            img[m < 0] = C_UNKNOWN
            img[m >= 50] = C_OCC
            air = (m >= 0) & (g < 0)
            img[air & (m < 50)] = AIR_FREE
            img[air & (m >= 50)] = AIR_OCC
            self._cache[k] = img[::-1]
        return self._cache[k]

    def layer(self, arr, t):
        return float(np.interp(t, self.cov_t, arr))

    def dgoal_of(self, i, t):
        g = self.dgoal[(self.dgoal[:, 1] == i) & (self.dgoal[:, 0] <= t)]
        if len(g) and g[-1, 2] > 0.5 and t - g[-1, 0] < 4:
            return g[-1, 3], g[-1, 4]
        return None


def draw_drone(d, cx, cy, r, col):
    for a in (math.pi / 4, 3 * math.pi / 4):
        dx, dy = r * math.cos(a), r * math.sin(a)
        d.line([(cx - dx, cy - dy), (cx + dx, cy + dy)], fill=(255, 255, 255), width=9)
        d.line([(cx - dx, cy - dy), (cx + dx, cy + dy)], fill=col, width=5)
    for a in (math.pi / 4, 3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4):
        px, py = cx + r * math.cos(a), cy + r * math.sin(a)
        rr = r * 0.42
        d.ellipse([px - rr, py - rr, px + rr, py + rr], outline=(255, 255, 255), width=3)
    d.ellipse([cx - r * 0.3, cy - r * 0.3, cx + r * 0.3, cy + r * 0.3], fill=col)


def map_panel(D, t):
    base = P3.map_panel(D, t)                      # карта, наземные роботы, отметки (уже уменьшено до MAP_W)
    im = base.resize((MAP_W * SS, base.size[1] * SS), Image.BILINEAR)
    d = ImageDraw.Draw(im)
    s = SS * MAP_W / (D.maps.shape[2] * D.res)
    for i in range(D.m):
        col = DRONE_COLORS[i % len(DRONE_COLORS)]
        tt, xs, ys, _ = D.dtr[i]
        if not len(tt) or t < tt[0]:
            continue
        j = np.searchsorted(tt, t, side='right')
        px, py = D.to_px(np.append(xs[:j], np.interp(t, tt, xs)), np.append(ys[:j], np.interp(t, tt, ys)))
        if len(px) > 1:
            pts = list(zip(px, py))
            for a in range(0, len(pts) - 1, 2):          # пунктир: путь по воздуху
                d.line(pts[a:a + 2], fill=col, width=4)
        g = D.dgoal_of(i, t)
        if g:
            gx, gy = D.to_px(*g)
            d.rectangle([gx - 12, gy - 12, gx + 12, gy + 12], outline=col, width=4)
        draw_drone(d, px[-1], py[-1], 0.45 * s, col)
    return im.resize((MAP_W, base.size[1]), Image.LANCZOS)


def base_frame(D, t, title, caption, speed=None):
    im = Image.new('RGB', (W, H), BG)
    im.paste(map_panel(D, t), (MAP_X, MAP_Y))
    d = ImageDraw.Draw(im)
    boxed(d, (30, 24), 'Simulation', F['s36'])
    if speed:
        w = d.textlength(speed, font=F['s36'])
        d.text((W - 30 - w, 24), speed, font=F['s36'], fill=TEAL)
    tf = P3.fit_font(d, title, 1000, sizes=(44, 40, 36, 32, 28, 25, 22))
    d.text(((W - d.textlength(title, font=tf)) / 2, 84), title, font=tf, fill=(255, 255, 255))
    cov = D.coverage(t)
    d.text((36, 822), f'{cov:.0f}%' if cov >= 99.95 else f'{cov:.1f}%', font=F['s70'], fill=TEAL)
    d.text((40, 900), 'shared map coverage', font=F['s22'], fill=(150, 160, 170))
    P3.cov_panel(D, d, t)
    cf = P3.fit_font(d, caption, 1030, sizes=(30, 27, 24, 22, 20))
    d.text(((W - d.textlength(caption, font=cf)) / 2, 1032), caption, font=cf, fill=(255, 255, 255))
    # правая колонка: тепловизор дрона (сверху), люди, ловушки, слои карты, легенда
    th, r = P3.thermal_panel(D, t)
    d.text((RX, 144), f'THERMAL, drone_{r + 1}', font=font(16), fill=(190, 200, 210))
    im.paste(th, (RX, 168))
    d.rectangle([RX - 2, 166, RX + RW + 1, 168 + th.size[1] + 1], outline=(90, 100, 112), width=2)
    nf, nr, nd, nt = D.counters(t)
    y = 168 + th.size[1] + 18
    d.text((RX, y), 'REACHABLE PEOPLE FOUND', font=font(16), fill=(190, 200, 210))
    d.text((RX, y + 18), f'{nf}/{nr}', font=F['s70'], fill=CONF)
    y += 104
    d.text((RX, y), 'DECOYS REJECTED', font=font(16), fill=(190, 200, 210))
    d.text((RX, y + 18), f'{nd}/{nt}', font=F['s70'], fill=REJ)
    y += 104
    d.text((RX, y), 'MAP SEEN', font=font(16), fill=(190, 200, 210))
    y += 24
    for lab, val, col in (('from the air', D.layer(D.cov_air, t), AIR_FREE),
                          ('from the ground', D.layer(D.cov_ground, t), C_FREE)):
        d.rectangle([RX, y + 5, RX + 18, y + 23], fill=col, outline=(90, 100, 112))
        d.text((RX + 28, y), lab, font=font(19, bold=False), fill=(215, 222, 230))
        v = f'{val:.0f}%'
        d.text((RX + RW - d.textlength(v, font=F['s22']), y - 1), v, font=F['s22'], fill=(255, 255, 255))
        y += 32
    y += 6
    draw_drone(d, RX + 12, y + 12, 10, DRONE_COLORS[0])
    d.text((RX + 36, y + 2), f'drone ({D.backend.upper()})', font=font(17, bold=False), fill=(215, 222, 230))
    y += 30
    d.ellipse([RX + 3, y + 3, RX + 21, y + 21], fill=ROBOT_COLORS[0])
    d.text((RX + 36, y + 2), 'ground robot', font=font(17, bold=False), fill=(215, 222, 230))
    return im


def outro(logo):
    im = Image.new('RGB', (W, H), BG)
    lg = Image.open(logo).convert('RGBA').resize((640, 640), Image.LANCZOS)
    im.paste(lg, (220, 90), lg)
    d = ImageDraw.Draw(im)
    boxed(d, (30, 24), 'Simulation', F['s36'])
    centered(d, 790, 'Bekai Vision', F['s70'], (255, 255, 255))
    centered(d, 890, 'RescueSwarm  |  Phase 5', F['s44'], TEAL)
    return im


def main():
    raw, logo, out = sys.argv[1:4]
    D = Data(raw)
    T = D.t_end - D.t0
    speed = max(1, math.ceil(T / P3.TL_MAX_S))
    n_tl = int(round(T / speed * FPS))
    n_intro, n_hold, n_out = int(P3.INTRO_S * FPS), int(P3.END_HOLD_S * FPS), int(P3.OUTRO_S * FPS)
    c = D.check
    print(f'seed {D.seed}: {T:.0f} сим. с, x{speed}, покрытие {D.coverage(D.t_end):.1f} %, дронов {D.m} ({D.backend})')
    title_res = (f'One map: people found {c["confirmed_reachable"]}/{c["reachable"]}, '
                 f'decoys rejected {c["decoys_rejected"]}/{c["decoys"]}')
    flight = 'PX4 autopilot' if D.backend == 'px4' else 'simple flight model'
    cap_a = f'2 drones ({flight}) map from above (blue), 3 ground robots from inside'
    cap_b = 'Drones spot warm targets from above; ground robots drive close to confirm'
    cap_c = 'One shared map, people confirmed up close, decoys rejected'
    ff = subprocess.Popen(['ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}',
                           '-r', str(FPS), '-i', '-', '-c:v', 'libx264', '-preset', 'slow', '-crf', '16', '-pix_fmt', 'yuv420p',
                           '-r', str(FPS), '-movflags', '+faststart', '-an', out], stdin=subprocess.PIPE)
    put = lambda im: ff.stdin.write(im.tobytes())     # noqa: E731
    first = base_frame(D, D.t_end, title_res, f'Result after {T:.0f} s of simulation, no human control, seed {D.seed}')
    for _ in range(n_intro):
        put(first)
    for k in range(n_tl):
        t = D.t0 + k * speed / FPS
        frac = k / max(1, n_tl - 1)
        cap = cap_a if frac < 0.35 else cap_b if frac < 0.7 else cap_c
        im = base_frame(D, t, 'Mixed swarm: exploration and search, timelapse', cap, speed=f'x{speed}  t = {t - D.t0:.0f} s')
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

#!/usr/bin/env python3
"""Сборка видео Фазы 4 из записи прогона на SLAM (внутри контейнера, без окон).

  phase4_render.py RAW_DIR LOGO.png OUT.mp4
RAW_DIR: rec.npz (phase4_rec.py: pose = оценка SLAM, gt = истинная поза), run/{coverage.csv,gt.json,check.json}, seed.txt.
Берёт всё общее из phase3_render (карта, отметки людей, тепловой кадр, график покрытия). Новое: истинный путь тонкой
серой линией, путь по SLAM цветной, справа текущая ошибка позы каждого робота |SLAM - истина|.
"""
import math
import os
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import phase3_render as P3  # noqa: E402
from phase2_render import (BG, FPS, H, ROBOT_COLORS, TEAL, W, F, boxed, centered, font)  # noqa: E402
from phase3_render import CONF, REJ, RX, RW, MAP_X, MAP_Y, MAP_W, SS, draw_mark  # noqa: E402

GREY = (170, 176, 184)


class Data(P3.Data):
    def __init__(self, raw):
        super().__init__(raw)
        z = np.load(os.path.join(raw, 'rec.npz'))
        g = z['gt']
        self.gt = {}
        for i in range(self.n):
            p = g[g[:, 1] == i]
            self.gt[i] = (p[:, 0], p[:, 2], p[:, 3])

    def err(self, i, t):
        """Расстояние между позой SLAM и истинной позой в момент t (м)."""
        tt, xs, ys, _ = self.tr[i]
        gt, gx, gy = self.gt[i]
        if t < tt[0] or t < gt[0]:
            return 0.0
        return float(math.hypot(np.interp(t, tt, xs) - np.interp(t, gt, gx), np.interp(t, tt, ys) - np.interp(t, gt, gy)))


def map_panel(D, t):
    rgb = D.map_at(t)
    mh = int(round(D.map_h))
    im = Image.fromarray(rgb).resize((MAP_W * SS, mh * SS), Image.BILINEAR)
    d = ImageDraw.Draw(im)
    for i in range(D.n):                                   # истинный путь: тонкая серая линия под цветными
        gt, gx, gy = D.gt[i]
        j = np.searchsorted(gt, t, side='right')
        if j > 1:
            px, py = D.to_px(gx[:j], gy[:j])
            d.line(list(zip(px, py)), fill=GREY, width=2, joint='curve')
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
            gx_, gy_ = D.to_px(*g)
            d.polygon([(gx_, gy_ - 14), (gx_ + 14, gy_), (gx_, gy_ + 14), (gx_ - 14, gy_)], outline=col, width=4)
        yaw = float(np.interp(t, tt, np.unwrap(yaws)))
        r = 0.3 * SS * MAP_W / (D.maps.shape[2] * D.res)
        d.ellipse([cx - r - 3, cy - r - 3, cx + r + 3, cy + r + 3], fill=(255, 255, 255))
        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=col)
        d.line([(cx, cy), (cx + 1.6 * r * math.cos(yaw), cy - 1.6 * r * math.sin(yaw))], fill=(255, 255, 255), width=5)
        gt, gx, gy = D.gt[i]                               # истинная позиция: серое кольцо
        if t >= gt[0]:
            tx, ty = D.to_px(np.interp(t, gt, gx), np.interp(t, gt, gy))
            d.ellipse([tx - r - 4, ty - r - 4, tx + r + 4, ty + r + 4], outline=GREY, width=3)
    s = SS * MAP_W / (D.maps.shape[2] * D.res)
    for mid, st, x, y, st_t in D.marks_at(t):
        cx, cy = D.to_px(x, y)
        if not (30 < cx < MAP_W * SS - 30 and 30 < cy < D.map_h * SS - 30):
            continue
        age = t - st_t
        if 0 <= age < 8.0:
            rr = 0.42 * s * (1 + 1.6 * age / 8.0)
            d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], outline=P3.STATUS_COL[st], width=6)
        draw_mark(d, cx, cy, st, 0.42 * s)
    return im.resize((MAP_W, mh), Image.LANCZOS)


def base_frame(D, t, title, caption, speed=None):
    im = Image.new('RGB', (W, H), BG)
    im.paste(map_panel(D, t), (MAP_X, MAP_Y))
    d = ImageDraw.Draw(im)
    boxed(d, (30, 24), 'Simulation', F['s36'])
    if speed:
        w = d.textlength(speed, font=F['s36'])
        d.text((W - 30 - w, 24), speed, font=F['s36'], fill=TEAL)
    tf = P3.fit_font(d, title, 1000)
    d.text(((W - d.textlength(title, font=tf)) / 2, 84), title, font=tf, fill=(255, 255, 255))
    d.text((36, 822), f'{D.coverage(t):.1f}%', font=F['s70'], fill=TEAL)
    d.text((40, 900), 'map coverage', font=F['s22'], fill=(150, 160, 170))
    P3.cov_panel(D, d, t)
    cf = P3.fit_font(d, caption, 1030, sizes=(30, 27, 24))
    d.text(((W - d.textlength(caption, font=cf)) / 2, 1032), caption, font=cf, fill=(255, 255, 255))
    # правая колонка: тепловизор, люди, ошибка позы, легенда
    th, r = P3.thermal_panel(D, t)
    d.text((RX, 144), f'THERMAL  ground_{r + 1}', font=font(16), fill=(190, 200, 210))
    im.paste(th, (RX, 168))
    d.rectangle([RX - 2, 166, RX + RW + 1, 168 + th.size[1] + 1], outline=(90, 100, 112), width=2)
    nf, nr, nd, nt = D.counters(t)
    y = 168 + th.size[1] + 22
    d.text((RX, y), 'REACHABLE PEOPLE FOUND', font=font(16), fill=(190, 200, 210))
    d.text((RX, y + 20), f'{nf}/{nr}', font=F['s70'], fill=CONF)
    y += 112
    d.text((RX, y), 'DECOYS REJECTED', font=font(16), fill=(190, 200, 210))
    d.text((RX, y + 20), f'{nd}/{nt}', font=F['s70'], fill=REJ)
    y += 112
    d.text((RX, y), 'POSE ERROR  |SLAM - TRUE|', font=font(16), fill=(190, 200, 210))
    y += 26
    for i in range(D.n):
        col = ROBOT_COLORS[i % 3]
        d.ellipse([RX, y + 6, RX + 18, y + 24], fill=col)
        d.text((RX + 28, y), f'ground_{i + 1}', font=font(20, bold=False), fill=(215, 222, 230))
        e = f'{D.err(i, t):.2f} m'
        d.text((RX + RW - d.textlength(e, font=F['s22']), y - 1), e, font=F['s22'], fill=(255, 255, 255))
        y += 34
    y += 8
    d.line([(RX, y + 12), (RX + 34, y + 12)], fill=GREY, width=3)
    d.text((RX + 44, y + 2), 'true path', font=font(17, bold=False), fill=(215, 222, 230))
    y += 28
    d.line([(RX, y + 12), (RX + 34, y + 12)], fill=ROBOT_COLORS[0], width=6)
    d.text((RX + 44, y + 2), 'SLAM path', font=font(17, bold=False), fill=(215, 222, 230))
    return im


def outro(logo):
    im = Image.new('RGB', (W, H), BG)
    lg = Image.open(logo).convert('RGBA').resize((640, 640), Image.LANCZOS)
    im.paste(lg, (220, 90), lg)
    d = ImageDraw.Draw(im)
    boxed(d, (30, 24), 'Simulation', F['s36'])
    centered(d, 790, 'Bekai Vision', F['s70'], (255, 255, 255))
    centered(d, 890, 'RescueSwarm  |  Phase 4', F['s44'], TEAL)
    return im


def main():
    raw, logo, out = sys.argv[1:4]
    D = Data(raw)
    T = D.t_end - D.t0
    speed = max(1, math.ceil(T / P3.TL_MAX_S))
    n_tl = int(round(T / speed * FPS))
    n_intro, n_hold, n_out = int(P3.INTRO_S * FPS), int(P3.END_HOLD_S * FPS), int(P3.OUTRO_S * FPS)
    c = D.check
    emax = max(max(D.err(i, t) for t in np.arange(D.t0, D.t_end, 2.0)) for i in range(D.n))
    print(f'seed {D.seed}: {T:.0f} сим. с, x{speed}, покрытие {D.coverage(D.t_end):.1f} %, макс. ошибка позы {emax:.2f} м')
    title_res = (f'Robots localize with SLAM: people found {c["confirmed_reachable"]}/{c["reachable"]}, '
                 f'decoys rejected {c["decoys_rejected"]}/{c["decoys"]}')
    cap_a = 'No ground truth: each robot localizes itself with SLAM (colored path)'
    cap_b = 'Grey = true path from the simulator, used only to measure the error'
    cap_c = 'Thermal search on the SLAM map: person = confirmed, decoy = rejected'
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
        cap = cap_a if frac < 0.3 else cap_b if frac < 0.6 else cap_c
        im = base_frame(D, t, 'Exploration and search on SLAM, timelapse', cap, speed=f'x{speed}  t = {t - D.t0:.0f} s')
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

#!/usr/bin/env python3
"""Сборка видео Фазы 6 из записи прогона с обрывом связи (внутри контейнера, без окон).

  phase6_render.py RAW_DIR LOGO.png OUT.mp4
RAW_DIR: rec.npz (phase6_rec.py), run/{coverage.csv,gt.json,check.json,comms_eval.json,comms/links.csv,
comms/comms_params.json}, seed.txt.
Слева карта базы (что видит спасатель): роботы, отметки людей, антенна базы и радиосвязи. Справа карта,
которую знает сам ground_1 (во время обрыва она растёт, а у базы нет). Внизу график покрытия у базы и
«идеального» (если бы связь была всегда), обрыв закрашен. Общее (шрифты, цвета, отметки) из phase2/phase3_render.
"""
import csv
import json
import math
import os
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import phase3_render as P3  # noqa: E402
from phase2_render import (BG, C_FREE, C_OCC, C_UNKNOWN, FPS, H, ROBOT_COLORS, TEAL, W, F, boxed, centered, font)  # noqa: E402
from phase3_render import CONF, REJ, STATUS_COL, SS, draw_mark, match  # noqa: E402

BASE_X, BASE_Y, BASE_W = 20, 150, 610            # левая панель: карта базы
OWN_X, OWN_Y, OWN_W = 650, 178, 410              # правая панель: карта ground_1
INTRO_S, END_HOLD_S, CARD_S, OUTRO_S, TL_MAX_S = 3.0, 1.0, 4.0, 3.5, 48.0
RED = (255, 80, 80)
LINK = (45, 212, 191)
ANT = (255, 205, 60)
GREY = (150, 160, 170)
LINE = (90, 100, 112)
COL_BASE = TEAL
COL_IDEAL = (255, 170, 0)


def to_float(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float('nan')


class Data:
    def __init__(self, raw):
        z = np.load(os.path.join(raw, 'rec.npz'))
        self.maps, self.maps_own, self.map_t = z['maps'], z['maps_own'], z['map_t']
        self.res, self.ox, self.oy = float(z['meta'][0]), float(z['meta'][1]), float(z['meta'][2])
        self.pose, self.marks = z['pose'], z['marks']
        run = os.path.join(raw, 'run')
        rows = list(csv.DictReader(open(os.path.join(run, 'coverage.csv'))))
        self.cov_t = np.array([float(r['sim_time_s']) for r in rows])
        self.cov = 100 * np.array([to_float(r.get('coverage')) for r in rows])
        self.cov_ideal = 100 * np.array([to_float(r.get('coverage_collective')) for r in rows])
        self.has_ideal = bool(np.isfinite(self.cov_ideal).any())
        gt = json.load(open(os.path.join(run, 'gt.json')))
        self.reach = [p for p in gt['people'] if p.get('reachable', True)]
        self.decoys = gt['decoys']
        self.check = json.load(open(os.path.join(run, 'check.json')))
        sp = os.path.join(raw, 'seed.txt')
        self.seed = open(sp).read().strip() if os.path.exists(sp) else '?'
        # обрыв, параметры базы, время догона
        cp = json.load(open(os.path.join(run, 'comms', 'comms_params.json')))
        self.base_xy = (float(cp['base_xyz'][0]), float(cp['base_xyz'][1]))
        outs = cp.get('outages') or []
        ev = {}
        try:
            ev = json.load(open(os.path.join(run, 'comms_eval.json')))
        except Exception:  # noqa: BLE001
            pass
        self.node, self.o0, self.o1 = 'ground_1', None, None
        if outs:
            self.node, self.o0, self.o1 = outs[0][0], float(outs[0][1]), float(outs[0][2])
        self.resync = None
        for o in ev.get('outages') or []:
            if o.get('node') == self.node and o.get('resync_s') is not None:
                self.resync = float(o['resync_s'])
        self.cov_base_end = ev.get('coverage_base')
        self.cov_ideal_end = ev.get('coverage_collective')
        # радиосвязи: время -> список пар (a, b), где up == 1
        by_t = {}
        with open(os.path.join(run, 'comms', 'links.csv')) as f:
            for r in csv.DictReader(f):
                if to_float(r.get('up')) >= 0.5:
                    by_t.setdefault(float(r['t']), []).append((r['a'], r['b']))
                else:
                    by_t.setdefault(float(r['t']), [])
        self.link_t = np.array(sorted(by_t))
        self.links = [by_t[t] for t in self.link_t]
        self.n = int(self.pose[:, 1].max()) + 1
        self.tr = {}
        for i in range(self.n):
            p = self.pose[self.pose[:, 1] == i]
            self.tr[i] = (p[:, 0], p[:, 2], p[:, 3], p[:, 4])
        self.t0 = float(self.cov_t[0])
        self.t_end = float(min(self.cov_t[-1], self.map_t[-1]))
        self.map_h = self.maps.shape[1] / self.maps.shape[2]
        self.mark_t = np.unique(self.marks[:, 0]) if len(self.marks) else np.zeros(0)
        self._cache = {}

    # --- время ---
    def in_outage(self, t):
        return self.o0 is not None and self.o0 <= t < self.o1

    def coverage(self, t):
        return float(np.interp(t, self.cov_t, self.cov))

    def ideal(self, t):
        ok = np.isfinite(self.cov_ideal)
        return float(np.interp(t, self.cov_t[ok], self.cov_ideal[ok])) if ok.any() else float('nan')

    # --- карты ---
    def grid_rgb(self, arr, key):
        if key not in self._cache:
            if len(self._cache) > 6:
                self._cache.clear()
            img = np.empty(arr.shape + (3,), np.uint8)
            img[:] = C_FREE
            img[arr < 0] = C_UNKNOWN
            img[arr >= 50] = C_OCC
            self._cache[key] = img[::-1]
        return self._cache[key]

    def idx(self, t):
        return int(np.clip(np.searchsorted(self.map_t, t, side='right') - 1, 0, len(self.map_t) - 1))

    def to_px(self, x, y, w):
        s = SS * w / (self.maps.shape[2] * self.res)
        return (np.asarray(x) - self.ox) * s, (self.maps.shape[1] * self.res - (np.asarray(y) - self.oy)) * s

    def pos_at(self, i, t):
        tt, xs, ys, yaws = self.tr[i]
        return float(np.interp(t, tt, xs)), float(np.interp(t, tt, ys)), float(np.interp(t, tt, np.unwrap(yaws)))

    def links_at(self, t):
        if not len(self.link_t):
            return []
        k = int(np.searchsorted(self.link_t, t, side='right') - 1)
        return self.links[max(k, 0)]

    def marks_at(self, t):
        if not len(self.mark_t):
            return []
        k = int(np.searchsorted(self.mark_t, t, side='right') - 1)
        if k < 0:
            return []
        r = self.marks[self.marks[:, 0] == self.mark_t[k]]
        return [(int(a[1]), int(a[2]), a[3], a[4], a[5]) for a in r]

    def found(self, t):
        conf = [(m[2], m[3]) for m in self.marks_at(t) if m[1] == 1]
        return len(match(conf, [(p['x'], p['y']) for p in self.reach])), len(self.reach)


def draw_robot(d, D, i, t, w, show_trail=True):
    col = ROBOT_COLORS[i % 3]
    tt, xs, ys, _ = D.tr[i]
    if t < tt[0]:
        return None
    j = np.searchsorted(tt, t, side='right')
    px, py = D.to_px(np.append(xs[:j], np.interp(t, tt, xs)), np.append(ys[:j], np.interp(t, tt, ys)), w)
    if show_trail and len(px) > 1:
        d.line(list(zip(px, py)), fill=col, width=5, joint='curve')
    cx, cy = px[-1], py[-1]
    yaw = D.pos_at(i, t)[2]
    r = 0.3 * SS * w / (D.maps.shape[2] * D.res)
    d.ellipse([cx - r - 3, cy - r - 3, cx + r + 3, cy + r + 3], fill=(255, 255, 255))
    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=col)
    d.line([(cx, cy), (cx + 1.6 * r * math.cos(yaw), cy - 1.6 * r * math.sin(yaw))], fill=(255, 255, 255), width=5)
    return cx, cy


def draw_antenna(d, cx, cy, s):
    """Антенна базы: мачта, перекладина и две дуги сигнала."""
    h = 0.9 * s
    d.line([(cx, cy), (cx, cy - h)], fill=(255, 255, 255), width=9)
    d.line([(cx, cy), (cx, cy - h)], fill=ANT, width=5)
    d.polygon([(cx - h * 0.35, cy + 2), (cx + h * 0.35, cy + 2), (cx, cy - h * 0.35)], fill=ANT, outline=(255, 255, 255))
    for k in (0.45, 0.8):
        rr = h * k
        d.arc([cx - rr, cy - h - rr, cx + rr, cy - h + rr], 215, 325, fill=ANT, width=4)
    d.ellipse([cx - 7, cy - h - 7, cx + 7, cy - h + 7], fill=(255, 255, 255))


def badge(d, cx, cy, text, f):
    w = d.textlength(text, font=f)
    asc, desc = f.getmetrics()
    x0, y0 = cx - w / 2 - 14, cy - (asc + desc) / 2 - 8
    d.rounded_rectangle([x0, y0, x0 + w + 28, y0 + asc + desc + 16], radius=14, fill=RED, outline=(255, 255, 255), width=3)
    d.text((x0 + 14, y0 + 8), text, font=f, fill=(255, 255, 255))


def base_panel(D, t):
    k = D.idx(t)
    w, h = BASE_W, int(round(BASE_W * D.map_h))
    im = Image.fromarray(D.grid_rgb(D.maps[k], ('b', k))).resize((w * SS, h * SS), Image.BILINEAR)
    d = ImageDraw.Draw(im)
    s = SS * w / (D.maps.shape[2] * D.res)
    pos = {f'ground_{i + 1}': D.to_px(*D.pos_at(i, t)[:2], w) for i in range(D.n) if t >= D.tr[i][0][0]}
    bx, by = D.to_px(*D.base_xy, w)
    pos['base'] = (float(bx), float(by))
    for a, b in D.links_at(t):                       # радиосвязи этой секунды
        if a in pos and b in pos:
            d.line([pos[a], pos[b]], fill=(255, 255, 255), width=9)
            d.line([pos[a], pos[b]], fill=LINK, width=5)
    for mid, st, x, y, st_t in D.marks_at(t):
        cx, cy = D.to_px(x, y, w)
        if not (30 < cx < w * SS - 30 and 30 < cy < h * SS - 30):
            continue
        age = t - st_t
        if 0 <= age < 8.0:
            rr = 0.42 * s * (1 + 1.6 * age / 8.0)
            d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], outline=STATUS_COL[st], width=6)
        draw_mark(d, cx, cy, st, 0.42 * s)
    pts = {}
    for i in range(D.n):
        c = draw_robot(d, D, i, t, w)
        if c:
            pts[f'ground_{i + 1}'] = c
    draw_antenna(d, bx, by, 1.5 * s)
    if D.in_outage(t) and D.node in pts:
        cx, cy = pts[D.node]
        badge(d, cx, cy - 0.9 * s - 24, 'нет связи', font(34))
    return im.resize((w, h), Image.LANCZOS)


def own_panel(D, t):
    k = D.idx(t)
    w, h = OWN_W, int(round(OWN_W * D.map_h))
    im = Image.fromarray(D.grid_rgb(D.maps_own[k], ('o', k))).resize((w * SS, h * SS), Image.BILINEAR)
    d = ImageDraw.Draw(im)
    i = int(D.node.split('_')[1]) - 1
    if 0 <= i < D.n:
        draw_robot(d, D, i, t, w)
    return im.resize((w, h), Image.LANCZOS)


def chart(D, d, t, x0=100, y0=790, w=940, h=140):
    d.line([(x0, y0 + h), (x0 + w, y0 + h)], fill=LINE, width=2)
    d.line([(x0, y0), (x0, y0 + h)], fill=LINE, width=2)
    tmax = D.t_end - D.t0
    for lvl in (50, 100):
        yy = y0 + h - h * lvl / 100
        d.line([(x0, yy), (x0 + w, yy)], fill=(50, 60, 72), width=1)
        d.text((x0 - 8 - d.textlength(f'{lvl}', font=F['s18']), yy - 11), f'{lvl}', font=F['s18'], fill=(130, 140, 150))
    if D.o0 is not None:                              # обрыв закрашен
        xa, xb = x0 + w * (D.o0 - D.t0) / tmax, x0 + w * (min(D.o1, D.t_end) - D.t0) / tmax
        d.rectangle([xa, y0, xb, y0 + h], fill=(70, 28, 32))
        lab = f'обрыв {D.node}'
        d.text(((xa + xb) / 2 - d.textlength(lab, font=F['s18']) / 2, y0 + 4), lab, font=F['s18'], fill=(255, 140, 140))
    m = D.cov_t <= t
    xs = x0 + w * (D.cov_t[m] - D.t0) / tmax
    for arr, col, wd in ((D.cov_ideal, COL_IDEAL, 3), (D.cov, COL_BASE, 5)):
        ys = y0 + h - h * arr[m] / 100
        pts = [(x, y) for x, y in zip(xs, ys) if math.isfinite(y)]
        if len(pts) > 1:
            d.line(pts, fill=col, width=wd, joint='curve')
    cx = x0 + w * (t - D.t0) / tmax
    d.line([(cx, y0), (cx, y0 + h)], fill=(255, 255, 255), width=1)
    cy = y0 + h - h * D.coverage(t) / 100
    d.ellipse([cx - 8, cy - 8, cx + 8, cy + 8], fill=(255, 255, 255))
    d.text((x0, y0 + h + 4), 'время симуляции', font=F['s18'], fill=(130, 140, 150))
    end = f'{tmax:.0f} с'
    d.text((x0 + w - d.textlength(end, font=F['s18']), y0 + h + 4), end, font=F['s18'], fill=(130, 140, 150))
    # подписи кривых
    lx = x0 + 250
    d.line([(lx, y0 + h + 15), (lx + 34, y0 + h + 15)], fill=COL_BASE, width=5)
    d.text((lx + 42, y0 + h + 4), f'у базы {D.coverage(t):.0f}%', font=F['s18'], fill=(215, 222, 230))
    if D.has_ideal:
        lx += 220
        d.line([(lx, y0 + h + 15), (lx + 34, y0 + h + 15)], fill=COL_IDEAL, width=3)
        d.text((lx + 42, y0 + h + 4), f'всего собрано роботами {D.ideal(t):.0f}%', font=F['s18'], fill=(215, 222, 230))


def side_info(D, d, t):
    i = int(D.node.split('_')[1]) - 1
    d.text((OWN_X, OWN_Y - 30), f'что знает {D.node}', font=font(22), fill=(215, 222, 230))
    own_h = int(round(OWN_W * D.map_h))
    d.rectangle([OWN_X - 2, OWN_Y - 2, OWN_X + OWN_W + 1, OWN_Y + own_h + 1], outline=RED if D.in_outage(t) else LINE, width=3)
    d.text((BASE_X, BASE_Y - 34), 'что видит база (спасатель)', font=font(22), fill=(215, 222, 230))
    y = OWN_Y + own_h + 14
    nf, nr = D.found(t)
    d.text((OWN_X, y), 'ЛЮДИ НАЙДЕНЫ НА БАЗЕ', font=font(16), fill=(190, 200, 210))
    d.text((OWN_X, y + 18), f'{nf}/{nr}', font=F['s70'], fill=CONF)
    y += 98
    # условные обозначения
    d.ellipse([OWN_X + 3, y + 3, OWN_X + 21, y + 21], fill=ROBOT_COLORS[0])
    d.text((OWN_X + 36, y + 2), 'наземный робот', font=font(17, bold=False), fill=(215, 222, 230))
    y += 30
    draw_antenna(d, OWN_X + 12, y + 26, 11)
    d.text((OWN_X + 36, y + 2), 'база (антенна)', font=font(17, bold=False), fill=(215, 222, 230))
    y += 34
    d.line([(OWN_X, y + 12), (OWN_X + 24, y + 12)], fill=LINK, width=5)
    d.text((OWN_X + 36, y + 2), 'радиосвязь сейчас', font=font(17, bold=False), fill=(215, 222, 230))
    y += 30
    draw_mark(d, OWN_X + 14, y + 12, 1, 11)
    d.text((OWN_X + 36, y + 2), 'найденный человек', font=font(17, bold=False), fill=(215, 222, 230))


def base_frame(D, t, title, caption, speed=None):
    im = Image.new('RGB', (W, H), BG)
    im.paste(base_panel(D, t), (BASE_X, BASE_Y))
    im.paste(own_panel(D, t), (OWN_X, OWN_Y))
    d = ImageDraw.Draw(im)
    boxed(d, (30, 24), 'Simulation', F['s36'])
    if speed:
        w = d.textlength(speed, font=F['s36'])
        d.text((W - 30 - w, 24), speed, font=F['s36'], fill=TEAL)
    tf = P3.fit_font(d, title, 1000, sizes=(44, 40, 36, 32, 28, 25, 22))
    d.text(((W - d.textlength(title, font=tf)) / 2, 84), title, font=tf, fill=(255, 255, 255))
    side_info(D, d, t)
    chart(D, d, t)
    cf = P3.fit_font(d, caption, 1030, sizes=(30, 27, 24, 22, 20))
    d.text(((W - d.textlength(caption, font=cf)) / 2, 1032), caption, font=cf, fill=(255, 255, 255))
    return im


def caption(D, t, speed):
    """Подпись по времени симуляции."""
    if D.o0 is None:
        return 'Три робота исследуют здание, база видит общую карту'
    win = max(8.0, 3.0 * speed)
    if t < D.o0:
        return 'Три робота исследуют здание, база видит общую карту'
    if t < D.o0 + win:
        return f'{D.node} потерял связь'
    if t < D.o1:
        return f'{D.node} исследует сам, база его данных не видит'
    if t < D.o1 + max(win, (D.resync or 0) + 2 * speed):
        return (f'связь вернулась: карта догоняет за {D.resync:.0f} с' if D.resync is not None
                else 'связь вернулась: карта догоняет')
    return 'карта базы догнала карту робота: данные дошли'


def end_card(D):
    im = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(im)
    boxed(d, (30, 24), 'Simulation', F['s36'])
    tt = 'Итог: связь рвётся, работа идёт'
    centered(d, 120, tt, P3.fit_font(d, tt, 960, sizes=(56, 50, 44)), (255, 255, 255))
    c = D.check
    rows = [(f'{c["confirmed_reachable"]}/{c["reachable"]}', 'людей подтверждено на базе', CONF)]
    if D.resync is not None:
        rows.append((f'{D.resync:.0f} с', 'карта базы догнала карту робота', TEAL))
    if D.o0 is not None:
        rows.append((f'{D.o1 - D.o0:.0f} с', f'{D.node} без связи, но исследовал', RED))
    cb = D.cov_base_end if D.cov_base_end is not None else D.coverage(D.t_end) / 100
    ci = D.cov_ideal_end if D.cov_ideal_end is not None else D.ideal(D.t_end) / 100
    rows.append((f'{100 * cb:.0f}%', 'покрытие у базы' + (f' (идеал {100 * ci:.0f}%)'
                                                              if ci is not None and math.isfinite(ci) else ''), COL_BASE))
    y = 290
    for big, lab, col in rows:
        d.text((90, y), big, font=F['s84'], fill=col)
        lf = P3.fit_font(d, lab, W - 460, sizes=(34, 30, 27, 24, 22))
        d.text((400, y + 30), lab, font=lf, fill=(215, 222, 230))
        y += 150
    return im


def outro(logo):
    im = Image.new('RGB', (W, H), BG)
    lg = Image.open(logo).convert('RGBA').resize((640, 640), Image.LANCZOS)
    im.paste(lg, (220, 90), lg)
    d = ImageDraw.Draw(im)
    boxed(d, (30, 24), 'Simulation', F['s36'])
    centered(d, 790, 'Bekai Vision', F['s70'], (255, 255, 255))
    centered(d, 890, 'RescueSwarm  |  Фаза 6', F['s44'], TEAL)
    return im


def main():
    raw, logo, out = sys.argv[1:4]
    D = Data(raw)
    T = D.t_end - D.t0
    speed = max(1, math.ceil(T / TL_MAX_S))
    n_tl = int(round(T / speed * FPS))
    n_intro, n_hold, n_card, n_out = int(INTRO_S * FPS), int(END_HOLD_S * FPS), int(CARD_S * FPS), int(OUTRO_S * FPS)
    c = D.check
    print(f'seed {D.seed}: {T:.0f} сим. с, x{speed}, покрытие у базы {D.coverage(D.t_end):.1f} %, '
          f'обрыв {D.node} {D.o0}-{D.o1}, догон {D.resync} с, кадров {n_intro + n_tl + n_hold + n_card + n_out}')
    title_res = f'Связь рвётся, работа идёт: людей найдено {c["confirmed_reachable"]}/{c["reachable"]}'
    ttl = 'Обрыв связи: робот работает, данные догоняют'
    ff = subprocess.Popen(['ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}',
                           '-r', str(FPS), '-i', '-', '-c:v', 'libx264', '-preset', 'slow', '-crf', '16', '-pix_fmt', 'yuv420p',
                           '-r', str(FPS), '-movflags', '+faststart', '-an', out], stdin=subprocess.PIPE)
    put = lambda im: ff.stdin.write(im.tobytes())     # noqa: E731
    final_cap = f'Итог через {T:.0f} с симуляции, без управления человеком, seed {D.seed}'
    first = base_frame(D, D.t_end, title_res, final_cap)
    for _ in range(n_intro):
        put(first)
    for k in range(n_tl):
        t = D.t0 + k * speed / FPS
        im = base_frame(D, t, ttl, caption(D, t, speed), speed=f'x{speed}  t = {t - D.t0:.0f} с')
        if k < 8:
            im = Image.blend(first, im, (k + 1) / 8)
        put(im)
    last = base_frame(D, D.t_end, ttl, caption(D, D.t_end, speed), speed=f'x{speed}  t = {T:.0f} с')
    for _ in range(n_hold):
        put(last)
    card = end_card(D)
    for k in range(n_card):
        put(Image.blend(last, card, min(1.0, (k + 1) / 6)) if k < 6 else card)
    ol = outro(logo)
    for k in range(n_out):
        put(Image.blend(card, ol, min(1.0, (k + 1) / 6)) if k < 6 else ol)
    ff.stdin.close()
    ff.wait()
    print('готово', out)


if __name__ == '__main__':
    main()

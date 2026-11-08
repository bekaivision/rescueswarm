#!/usr/bin/env python3
"""Замер YOLO (yolo11n / yolo11s, класс person) на кадрах из симулятора.

Запускается ТОЛЬКО в контейнере (scripts/yolo_bench.sh). Подкоманды:
  sanity         снимки позы Randy (лицом вверх, по центру, на полу) -> results/phase-3b/sanity/
  capture-bench  кадры бенч-мира: дальность x ракурс, >= N кадров на ячейку
  capture-scene  кадры сцены без людей (ложные цели, стены, завал) для ложных срабатываний
  eval           YOLO на сохранённых кадрах: CSV по ячейкам, summary.md, размеченные кадры
  latency        задержка инференса, мс/кадр, GPU, batch 1

Камера в бенче: отдельный сенсор camera (не робот) с теми же параметрами, что у rgb робота:
320x240, hfov 1.047, R8G8B8, высота 0.22 м (base_link на колёсах 0.08 + rgb_link z=0.14),
взгляд горизонтально, 20 Гц (на статичной сцене частота не влияет на картинку).
Мир собирается из rs_worlds.generate.world_sdf (тот же свет, пол, блок позы человека).
"""
import argparse
import csv
import glob
import json
import math
import os
import random
import re
import shutil
import subprocess
import sys
import threading
import time

import numpy as np

ROOT = os.environ.get('RS_ROOT', '/workspace')
OUT = os.path.join(ROOT, 'results', 'phase-3b')
TMP = '/tmp/yolo_bench'
WORLD = 'rescue'
TOPIC = '/bench/rgb'
W, H, HFOV = 320, 240, 1.047
FX = (W / 2.0) / math.tan(HFOV / 2.0)
CAM_Z = 0.22
NEAR = 0.1

RANGES = [1, 2, 3, 4, 6, 8]
ANGLES = [0, 45, 90, 135, 180]      # поворот тела относительно взгляда: 0 = ноги к камере
MODELS = ['yolo11n', 'yolo11s']
IMGSZ = [320, 640]
CONFS = [0.10, 0.25]


# человек: те же размеры, что в generate.py (меш лёжа, центр в начале модели)
BODY_BOX = (0.9105, 0.25, 0.36)   # полуразмеры x, y; высота z (только тело)
FULL_BOX = (0.9105, 0.682, 0.36)  # с руками


# ---------------------------------------------------------------- Gazebo

class Gz:
    """Клиент gz-transport: подписка на камеру и сервис set_pose."""

    def __init__(self, topic=TOPIC, world=WORLD):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.image_pb2 import Image
        from gz.msgs10.pose_pb2 import Pose
        from gz.transport13 import Node
        self.Pose, self.Boolean = Pose, Boolean
        self.node = Node()
        self.world = world
        self.lock = threading.Lock()
        self.count = 0
        self.last = None
        if not self.node.subscribe(Image, topic, self._cb):
            raise RuntimeError('не удалось подписаться на ' + topic)

    def _cb(self, msg):
        arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        with self.lock:
            self.last = arr[:, :, ::-1].copy()   # RGB -> BGR для cv2
            self.count += 1

    def wait_first(self, timeout=90):
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                if self.count:
                    return
            time.sleep(0.2)
        raise RuntimeError('камера не публикует кадры')

    def set_pose(self, name, x, y, z, yaw, pitch=0.0):
        req = self.Pose()
        req.name = name
        req.position.x, req.position.y, req.position.z = x, y, z
        cy, sy, cp, sp = math.cos(yaw / 2.0), math.sin(yaw / 2.0), math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        req.orientation.w, req.orientation.x = cy * cp, -sy * sp
        req.orientation.y, req.orientation.z = cy * sp, sy * cp
        for _try in range(5):
            ok, rep = self.node.request(f'/world/{self.world}/set_pose', req, self.Pose, self.Boolean, 3000)
            if ok and rep.data:
                return
            time.sleep(0.5)
        raise RuntimeError(f'set_pose {name} не удался (ok={ok})')

    def grab(self, skip=4, timeout=10):
        """Кадр, снятый уже после последнего set_pose: пропускаем skip кадров."""
        with self.lock:
            c0 = self.count
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                if self.count >= c0 + skip:
                    return self.last.copy()
            time.sleep(0.02)
        raise RuntimeError('нет новых кадров камеры')


def start_gz(sdf):
    log = open(os.path.join(TMP, 'gz.log'), 'w')
    p = subprocess.Popen(['gz', 'sim', '-s', '--headless-rendering', '-r', sdf],
                         stdout=log, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
    return p


def stop_gz(p):
    import signal
    try:
        os.killpg(p.pid, signal.SIGINT)
        p.wait(timeout=20)
    except Exception:
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except Exception:
            pass


# ---------------------------------------------------------------- миры

CAM_MODEL = f'''    <model name="bench_cam">
      <static>true</static>
      <pose>0 0 {CAM_Z} 0 0 0</pose>
      <link name="link">
        <sensor name="rgb" type="camera">
          <always_on>1</always_on>
          <update_rate>20</update_rate>
          <topic>{TOPIC}</topic>
          <camera>
            <horizontal_fov>{HFOV}</horizontal_fov>
            <image><width>{W}</width><height>{H}</height><format>R8G8B8</format></image>
            <clip><near>{NEAR}</near><far>50</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
'''


def _strip_models(sdf, prefix):
    return re.sub(r'    <model name="%s[^"]*">.*?\n    </model>\n' % prefix, '', sdf, flags=re.S)


def make_bench_world():
    """Пол + свет + один Randy (person_1) + камера. Без стен."""
    from rs_worlds import generate as G
    lay = dict(walls=[])
    w = dict(seed=0, size='bench', lay=lay, rubble=[], decoys=[],
             people=[dict(x=30.0, y=30.0, yaw=0.0)])
    sdf = G.world_sdf(w)
    sdf = _strip_models(sdf, 'walls')
    sdf = sdf.replace('  </world>', CAM_MODEL + '  </world>')
    os.makedirs(TMP, exist_ok=True)
    path = os.path.join(TMP, 'bench_world.sdf')
    with open(path, 'w') as f:
        f.write(sdf)
    return path


def make_scene_world(seed=1, size='medium'):
    """Мир, сгенерированный generate.py, без людей, плюс камера."""
    d = os.path.join(TMP, 'scene')
    shutil.rmtree(d, ignore_errors=True)
    subprocess.run([sys.executable, '-m', 'rs_worlds.generate', '--seed', str(seed),
                    '--out', d, '--size', size], check=True, stdout=subprocess.DEVNULL)
    with open(os.path.join(d, 'world.sdf')) as f:
        sdf = f.read()
    sdf = _strip_models(sdf, 'person_')
    sdf = sdf.replace('  </world>', CAM_MODEL + '  </world>')
    path = os.path.join(d, 'world_noppl.sdf')
    with open(path, 'w') as f:
        f.write(sdf)
    return path, d


# ------------------------------------------------------- геометрия проекции

def _box_corners(hx, hy, h, px, py, yaw):
    pts = []
    c, s = math.cos(yaw), math.sin(yaw)
    for sx in (-1, 1):
        for sy in (-1, 1):
            for z in (0.0, h):
                lx, ly = sx * hx, sy * hy
                pts.append((px + c * lx - s * ly, py + s * lx + c * ly, z))
    return pts


def project_box(corners, cam_x, cam_y, cam_yaw):
    """Проекция выпуклого бокса в пиксели -> (x0, y0, x1, y1) без обрезки по кадру или None."""
    c, s = math.cos(-cam_yaw), math.sin(-cam_yaw)
    cam = []
    for (x, y, z) in corners:
        dx, dy = x - cam_x, y - cam_y
        fwd = c * dx - s * dy
        left = s * dx + c * dy
        cam.append((fwd, left, z - CAM_Z))
    # рёбра куба: пары углов, отличающиеся одним битом индекса
    pts = []
    for i in range(8):
        for b in (1, 2, 4):
            j = i ^ b
            if j < i:
                continue
            a, bq = cam[i], cam[j]
            if a[0] < NEAR and bq[0] < NEAR:
                continue
            if a[0] < NEAR or bq[0] < NEAR:
                t = (NEAR - a[0]) / (bq[0] - a[0])
                m = tuple(a[k] + t * (bq[k] - a[k]) for k in range(3))
                if a[0] < NEAR:
                    a = m
                else:
                    bq = m
            pts += [a, bq]
    if not pts:
        return None
    us = [W / 2.0 - FX * p[1] / p[0] for p in pts]
    vs = [H / 2.0 - FX * p[2] / p[0] for p in pts]
    return min(us), min(vs), max(us), max(vs)


def clip_frac(b):
    """Доля площади бокса, попавшая в кадр."""
    if b is None:
        return 0.0
    x0, y0, x1, y1 = b
    a = max(x1 - x0, 1e-6) * max(y1 - y0, 1e-6)
    cx0, cy0, cx1, cy1 = max(x0, 0), max(y0, 0), min(x1, W), min(y1, H)
    if cx1 <= cx0 or cy1 <= cy0:
        return 0.0
    return (cx1 - cx0) * (cy1 - cy0) / a


def gt_regions(px, py, yaw, cam_yaw):
    body = project_box(_box_corners(BODY_BOX[0], BODY_BOX[1], BODY_BOX[2], px, py, yaw), 0, 0, cam_yaw)
    full = project_box(_box_corners(FULL_BOX[0], FULL_BOX[1], FULL_BOX[2], px, py, yaw), 0, 0, cam_yaw)
    return body, full


# ------------------------------------------------------------------- снимки

def _save(path, img):
    import cv2
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cv2.imwrite(path, img)


def cmd_sanity(a):
    """Randy рядом с камерой с нескольких сторон + вид сверху и сбоку (вторая камера не нужна:
    для вида сверху поднимаем и наклоняем саму камеру)."""
    sdf = make_bench_world()
    gz = start_gz(sdf)
    try:
        g = Gz()
        g.wait_first()
        out = os.path.join(OUT, os.environ.get('RS_SANITY_DIR', 'sanity'))
        os.makedirs(out, exist_ok=True)
        # человек в (3, 0), голова к +x (yaw 0). Камера у начала координат смотрит на +x.
        # 1) ноги к камере (голова от неё): человек на (3,0) yaw 0
        shots = [
            ('feet_to_cam_r3', (3, 0, 0.0), None),
            ('side_r3', (3, 0, math.pi / 2), None),
            ('head_to_cam_r3', (3, 0, math.pi), None),
            ('feet_to_cam_r1.5', (1.5, 0, 0.0), None),
            ('side_r1.5', (1.5, 0, math.pi / 2), None),
        ]
        for name, (x, y, yaw), _ in shots:
            g.set_pose('person_1', x, y, 0.0, yaw)
            g.set_pose('bench_cam', 0, 0, CAM_Z, 0)
            _save(os.path.join(out, f'{name}.png'), g.grab())
        # вид сверху (pitch 90 вниз, верх кадра = +x = голова) и сверху-сбоку под 45 градусов
        g.set_pose('person_1', 0, 0, 0.0, 0.0)
        g.set_pose('bench_cam', 0, 0, 3.0, 0.0, math.pi / 2)
        _save(os.path.join(out, 'top_down.png'), g.grab())
        g.set_pose('bench_cam', -2.0, -2.0, 2.0, math.pi / 4, math.radians(40))
        _save(os.path.join(out, 'oblique_head_right.png'), g.grab())
        print('sanity: кадры в', out)
    finally:
        stop_gz(gz)


# ------------------------------------------------------------- захват: бенч

def cmd_capture_bench(a):
    sdf = make_bench_world()
    gz = start_gz(sdf)
    rnd = random.Random(a.seed)
    out = os.path.join(OUT, 'yolo_bench', 'frames', 'bench')
    shutil.rmtree(out, ignore_errors=True)
    os.makedirs(out, exist_ok=True)
    meta = []
    try:
        g = Gz()
        g.wait_first()
        total = len(RANGES) * len(ANGLES) * a.frames
        k = 0
        t0 = time.time()
        for r in RANGES:
            for ang in ANGLES:
                for i in range(a.frames):
                    phi = rnd.uniform(-math.pi, math.pi)            # куда смотрит камера (свет меняется)
                    alpha = math.radians(rnd.uniform(-12, 12))      # смещение человека от оси взгляда
                    rr = r * rnd.uniform(0.97, 1.03)
                    beta = phi + alpha
                    px, py = rr * math.cos(beta), rr * math.sin(beta)
                    sign = 1 if (i % 2 == 0) else -1
                    rel = ang if ang in (0, 180) else sign * ang
                    rel += rnd.uniform(-8, 8) if ang not in (0, 180) else rnd.uniform(-5, 5)
                    yaw = beta + math.radians(rel)
                    g.set_pose('person_1', px, py, 0.0, yaw)
                    g.set_pose('bench_cam', 0, 0, CAM_Z, phi)
                    img = g.grab()
                    body, full = gt_regions(px, py, yaw, phi)
                    name = f'r{r}_a{ang}_{i:02d}'
                    _save(os.path.join(out, name + '.png'), img)
                    meta.append(dict(name=name, range=r, angle=ang, dist=rr, phi=phi, px=px, py=py,
                                     yaw=yaw, body=body, full=full, vis=clip_frac(full),
                                     vis_body=clip_frac(body)))
                    k += 1
                if k % 30 == 0:
                    print(f'bench {k}/{total} ({time.time() - t0:.0f} с)', flush=True)
        with open(os.path.join(out, 'meta.json'), 'w') as f:
            json.dump(meta, f)
        print('bench: кадров', len(meta))
    finally:
        stop_gz(gz)


# ------------------------------------------------------------- захват: сцена

def _ray_clear(occ, res, xmin, ymax, x0, y0, x1, y1):
    # сама цель занята в карте: луч обрываем в 0.5 м до неё
    dd = math.hypot(x1 - x0, y1 - y0)
    if dd > 0.5:
        k = (dd - 0.5) / dd
        x1, y1 = x0 + k * (x1 - x0), y0 + k * (y1 - y0)
    n = int(math.hypot(x1 - x0, y1 - y0) / (res * 0.5)) + 1
    for t in np.linspace(0, 1, n):
        x, y = x0 + t * (x1 - x0), y0 + t * (y1 - y0)
        i, j = int((x - xmin) / res), int((ymax - y) / res)
        if 0 <= j < occ.shape[0] and 0 <= i < occ.shape[1] and occ[j, i]:
            return False
    return True


def cmd_capture_scene(a):
    import yaml
    from scipy import ndimage
    out = os.path.join(OUT, 'yolo_bench', 'frames', 'scene')
    shutil.rmtree(out, ignore_errors=True)
    os.makedirs(out, exist_ok=True)
    meta = []
    for seed in [int(v) for v in a.seeds.split(',')]:
        sdf, d = make_scene_world(seed)
        with open(os.path.join(d, 'ground_truth.yaml')) as f:
            gt = yaml.safe_load(f)
        with open(os.path.join(d, 'gt_map.yaml')) as f:
            my = yaml.safe_load(f)
        res = my['resolution']
        xmin, ymin = my['origin'][0], my['origin'][1]
        with open(os.path.join(d, 'gt_map.pgm'), 'rb') as f:
            assert f.readline().strip() == b'P5'
            wd, ht = map(int, f.readline().split())
            f.readline()
            pgm = np.frombuffer(f.read(), np.uint8).reshape(ht, wd)
        ymax = ymin + ht * res
        occ = pgm < 128
        dist = ndimage.distance_transform_edt(~occ) * res
        b = gt['building']
        jj, ii = np.nonzero(dist >= 0.40)
        xs = xmin + (ii + 0.5) * res
        ys = ymax - (jj + 0.5) * res
        inside = ((xs > b['xmin'] + 0.3) & (xs < b['xmax'] - 0.3) &
                  (ys > b['ymin'] + 0.3) & (ys < b['ymax'] - 0.3))
        cand = np.nonzero(inside)[0]
        decoys = gt['decoys']
        rnd = random.Random(seed + 100)
        gzp = start_gz(sdf)
        try:
            g = Gz()
            g.wait_first()
            t0 = time.time()
            for k in range(a.frames):
                aimed = (k % 2 == 0)
                for _try in range(300):
                    idx = cand[rnd.randrange(len(cand))]
                    cx, cy = float(xs[idx]), float(ys[idx])
                    if aimed:
                        dc = rnd.choice(decoys)
                        dd = math.hypot(dc['x'] - cx, dc['y'] - cy)
                        if not (1.2 <= dd <= 5.0):
                            continue
                        if not _ray_clear(occ, res, xmin, ymax, cx, cy, dc['x'], dc['y']):
                            continue
                        phi = math.atan2(dc['y'] - cy, dc['x'] - cx) + math.radians(rnd.uniform(-18, 18))
                    else:
                        phi = rnd.uniform(-math.pi, math.pi)
                    break
                else:
                    phi = rnd.uniform(-math.pi, math.pi)
                g.set_pose('bench_cam', cx, cy, CAM_Z, phi)
                img = g.grab()
                inview = []
                for dc in decoys:
                    dx, dy = dc['x'] - cx, dc['y'] - cy
                    dd = math.hypot(dx, dy)
                    da = (math.atan2(dy, dx) - phi + math.pi) % (2 * math.pi) - math.pi
                    if dd < 8.0 and abs(da) < HFOV / 2.0 and _ray_clear(occ, res, xmin, ymax, cx, cy, dc['x'], dc['y']):
                        inview.append(dc['type'])
                name = f's{seed}_{k:03d}'
                _save(os.path.join(out, name + '.png'), img)
                meta.append(dict(name=name, seed=seed, x=cx, y=cy, phi=phi, aimed=aimed, decoys=inview))
            print(f'scene seed {seed}: {a.frames} кадров за {time.time() - t0:.0f} с', flush=True)
        finally:
            stop_gz(gzp)
    with open(os.path.join(out, 'meta.json'), 'w') as f:
        json.dump(meta, f)
    print('scene: кадров', len(meta))


# ------------------------------------------------------------------- YOLO

def weights(name):
    return f'/opt/yolo/{name}.pt'


def run_models(frames, models, imgszs):
    """-> {(model, imgsz): [[(conf, x0, y0, x1, y1), ...] по кадрам]} при conf 0.10, класс person."""
    from ultralytics import YOLO
    res = {}
    for m in models:
        net = YOLO(weights(m))
        for sz in imgszs:
            allb = []
            for s in range(0, len(frames), 32):
                rs = net.predict(frames[s:s + 32], imgsz=sz, conf=min(CONFS), classes=[0],
                                 verbose=False, device=0)
                for r in rs:
                    bx = r.boxes
                    allb.append([(float(c), *map(float, xy)) for c, xy in
                                 zip(bx.conf.cpu().numpy(), bx.xyxy.cpu().numpy())])
            res[(m, sz)] = allb
            print(f'{m} imgsz={sz}: готово', flush=True)
        del net
    return res


def _inside(b, region, pad=0.10):
    cx, cy = (b[1] + b[3]) / 2.0, (b[2] + b[4]) / 2.0
    x0, y0, x1, y1 = region
    w, h = x1 - x0, y1 - y0
    return x0 - pad * w <= cx <= x1 + pad * w and y0 - pad * h <= cy <= y1 + pad * h


def _iou(b, region):
    ax0, ay0, ax1, ay1 = b[1:5]
    bx0, by0, bx1, by1 = region
    iw, ih = min(ax1, bx1) - max(ax0, bx0), min(ay1, by1) - max(ay0, by0)
    if iw <= 0 or ih <= 0:
        return 0.0
    inter = iw * ih
    return inter / ((ax1 - ax0) * (ay1 - ay0) + (bx1 - bx0) * (by1 - by0) - inter)


def match(dets, full, vis):
    """Возвращает (hit, лучшая conf, число ложных боксов). Попадание: центр бокса внутри
    проекции бокса человека (с руками, запас 10%); если человек вне кадра, hit = None."""
    if vis < 0.15:   # человек почти не в кадре (камера внутри тела или за краем): ячейка не считается
        return None, 0.0, 0
    hits = [d for d in dets if _inside(d, full)]
    fps = len(dets) - len(hits)
    return (len(hits) > 0), (max(d[0] for d in hits) if hits else 0.0), fps


def load_frames(sub):
    import cv2
    d = os.path.join(OUT, 'yolo_bench', 'frames', sub)
    with open(os.path.join(d, 'meta.json')) as f:
        meta = json.load(f)
    frames = [cv2.imread(os.path.join(d, m['name'] + '.png')) for m in meta]
    return meta, frames


def draw(img, dets, region=None, scale=2):
    import cv2
    im = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    if region is not None:
        x0, y0, x1, y1 = [int(v * scale) for v in region]
        cv2.rectangle(im, (x0, y0), (x1, y1), (0, 200, 0), 1)
    for d in dets:
        x0, y0, x1, y1 = [int(v * scale) for v in d[1:5]]
        cv2.rectangle(im, (x0, y0), (x1, y1), (0, 0, 255), 2)
        cv2.putText(im, f'{d[0]:.2f}', (x0, max(12, y0 - 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    return im


def cmd_eval(a):
    import cv2
    base = os.path.join(OUT, 'yolo_bench')
    os.makedirs(base, exist_ok=True)
    bmeta, bframes = load_frames('bench')
    smeta, sframes = load_frames('scene') if os.path.isdir(os.path.join(base, 'frames', 'scene')) else ([], [])
    allf = bframes + sframes
    res = run_models(allf, MODELS, IMGSZ)
    nb = len(bframes)
    cells = {}      # (m, sz, conf) -> {(r, ang): dict}
    scene = {}      # (m, sz, conf) -> dict
    for (m, sz), dets_all in res.items():
        for conf in CONFS:
            filt = [[d for d in fr if d[0] >= conf] for fr in dets_all]
            agg = {}
            for meta, dets in zip(bmeta, filt[:nb]):
                hit, c, fps = match(dets, meta['full'], meta['vis'])
                e = agg.setdefault((meta['range'], meta['angle']),
                                   dict(n=0, n_vis=0, hits=0, confs=[], fp=0, fp_frames=0, vis=[]))
                e['n'] += 1
                e['vis'].append(meta['vis'])
                e['fp'] += fps
                e['fp_frames'] += 1 if fps else 0
                if hit is not None:
                    e['n_vis'] += 1
                    e['hits'] += 1 if hit else 0
                    if hit:
                        e['confs'].append(c)
            cells[(m, sz, conf)] = agg
            if smeta:
                fr = filt[nb:]
                tot = sum(1 for d in fr if d)
                by = {}
                for meta, d in zip(smeta, fr):
                    for t in set(meta['decoys']) or {'none'}:
                        e = by.setdefault(t, [0, 0])
                        e[0] += 1
                        e[1] += 1 if d else 0
                scene[(m, sz, conf)] = dict(n=len(fr), fp_frames=tot, boxes=sum(len(d) for d in fr), by=by)
    # CSV по ячейкам
    for (m, sz, conf), agg in cells.items():
        p = os.path.join(base, f'cells_{m}_{sz}_c{conf:.2f}.csv')
        with open(p, 'w', newline='') as f:
            wr = csv.writer(f)
            wr.writerow(['range_m', 'angle_deg', 'frames', 'frames_person_visible', 'detected',
                         'det_rate', 'mean_conf_on_hits', 'fp_boxes', 'fp_frames', 'mean_visible_frac'])
            for r in RANGES:
                for ang in ANGLES:
                    e = agg[(r, ang)]
                    rate = e['hits'] / e['n_vis'] if e['n_vis'] else float('nan')
                    mc = float(np.mean(e['confs'])) if e['confs'] else float('nan')
                    wr.writerow([r, ang, e['n'], e['n_vis'], e['hits'], f'{rate:.3f}', f'{mc:.3f}',
                                 e['fp'], e['fp_frames'], f'{np.mean(e["vis"]):.2f}'])
    if scene:
        with open(os.path.join(base, 'scene_fp.csv'), 'w', newline='') as f:
            wr = csv.writer(f)
            wr.writerow(['model', 'imgsz', 'conf', 'frames', 'frames_with_person_box', 'fp_frame_rate',
                         'boxes', 'subset', 'subset_frames', 'subset_fp_frames', 'subset_fp_rate'])
            for (m, sz, conf), s in scene.items():
                for t, (n, k) in sorted(s['by'].items()):
                    wr.writerow([m, sz, conf, s['n'], s['fp_frames'], f'{s["fp_frames"] / s["n"]:.3f}',
                                 s['boxes'], t, n, k, f'{k / n:.3f}'])
    # размеченные кадры
    ann = os.path.join(base, 'annotated')
    shutil.rmtree(ann, ignore_errors=True)
    os.makedirs(ann, exist_ok=True)
    best = ('yolo11n', 640)
    pick = [(r, ang) for r in (1, 3, 6, 8) for ang in (0, 90, 180)]
    for (r, ang) in pick:
        for i, (meta, img) in enumerate(zip(bmeta, bframes)):
            if meta['range'] == r and meta['angle'] == ang:
                dets = [d for d in res[best][i] if d[0] >= 0.25]
                cv2.imwrite(os.path.join(ann, f'{best[0]}_{best[1]}_r{r}_a{ang}.png'),
                            draw(img, dets, meta['full']))
                break
    if smeta:
        k = 0
        for i, (meta, img) in enumerate(zip(smeta, sframes)):
            dets = [d for d in res[best][nb + i] if d[0] >= 0.10]
            if dets and k < 6:
                cv2.imwrite(os.path.join(ann, f'scene_fp_{meta["name"]}_{"_".join(meta["decoys"]) or "none"}.png'),
                            draw(img, dets))
                k += 1
        for i, (meta, img) in enumerate(zip(smeta, sframes)):
            if meta['decoys'] and not res[best][nb + i] and i < 60:
                cv2.imwrite(os.path.join(ann, f'scene_ok_{meta["name"]}_{"_".join(meta["decoys"])}.png'),
                            draw(img, []))
                break
    with open(os.path.join(base, 'raw_detections.json'), 'w') as f:
        json.dump({f'{m}_{sz}': dets for (m, sz), dets in res.items()}, f)
    write_summary(base, cells, scene)


def write_summary(base, cells, scene):
    L = []
    w = L.append
    w('# Замер YOLO на кадрах симулятора')
    w('')
    w('Камера: отдельный сенсор `camera` 320x240, hfov 1.047, высота 0.22 м, взгляд горизонтально. '
      'Класс person, `device=0`. Попадание: центр бокса внутри проекции бокса человека (с руками, запас 10%). '
      'Ложный бокс: person-бокс вне этой области. Ракурс: 0 = ноги к камере, 180 = голова к камере.')
    w('')
    w('## Сводка по конфигурациям (все ячейки бенча)')
    w('')
    w('| модель | imgsz | conf | доля обнаружений | средняя conf | ложных боксов на кадр (бенч) | ложных кадров на сцене без людей |')
    w('|---|---|---|---|---|---|---|')
    summ = {}
    for key, agg in sorted(cells.items()):
        nv = sum(e['n_vis'] for e in agg.values())
        hits = sum(e['hits'] for e in agg.values())
        cf = [c for e in agg.values() for c in e['confs']]
        fp = sum(e['fp'] for e in agg.values())
        nf = sum(e['n'] for e in agg.values())
        s = scene.get(key)
        sfp = f'{s["fp_frames"]}/{s["n"]} ({100.0 * s["fp_frames"] / s["n"]:.1f}%)' if s else 'нет'
        summ[key] = hits / nv if nv else 0
        w(f'| {key[0]} | {key[1]} | {key[2]:.2f} | {hits}/{nv} = {100.0 * hits / nv:.1f}% | '
          f'{np.mean(cf) if cf else 0:.2f} | {fp / nf:.3f} | {sfp} |')
    w('')
    for key, agg in sorted(cells.items()):
        w(f'## {key[0]}, imgsz {key[1]}, conf {key[2]:.2f}: доля обнаружений (дальность x ракурс)')
        w('')
        w('| дальность, м | ' + ' | '.join(f'{a}°' for a in ANGLES) + ' |')
        w('|---|' + '---|' * len(ANGLES))
        for r in RANGES:
            row = []
            for ang in ANGLES:
                e = agg[(r, ang)]
                row.append(f'{e["hits"]}/{e["n_vis"]}' + (f' ({e["hits"] / e["n_vis"] * 100:.0f}%)' if e['n_vis'] else ''))
            w(f'| {r} | ' + ' | '.join(row) + ' |')
        w('')
        w('Средняя conf на попаданиях / ложных боксов на ячейку:')
        w('')
        w('| дальность, м | ' + ' | '.join(f'{a}°' for a in ANGLES) + ' |')
        w('|---|' + '---|' * len(ANGLES))
        for r in RANGES:
            row = []
            for ang in ANGLES:
                e = agg[(r, ang)]
                row.append(f'{np.mean(e["confs"]) if e["confs"] else 0:.2f} / {e["fp"]}')
            w(f'| {r} | ' + ' | '.join(row) + ' |')
        w('')
    e0 = next(iter(cells.values()))
    w('## Видимая доля человека в кадре (средняя по ячейке)')
    w('')
    w('| дальность, м | ' + ' | '.join(f'{a}°' for a in ANGLES) + ' |')
    w('|---|' + '---|' * len(ANGLES))
    for r in RANGES:
        w(f'| {r} | ' + ' | '.join(f'{np.mean(e0[(r, a)]["vis"]):.2f}' for a in ANGLES) + ' |')
    w('')
    if scene:
        w('## Ложные срабатывания на сцене без людей')
        w('')
        w('Кадры из миров generate.py (seed 1-4) без людей: половина смотрит на ложные цели, половина в случайную сторону. '
          'Подмножество = типы ложных целей в поле зрения кадра ("none" = ни одной).')
        w('')
        w('| модель | imgsz | conf | подмножество | кадров | с person-боксом | доля |')
        w('|---|---|---|---|---|---|---|')
        for key, s in sorted(scene.items()):
            w(f'| {key[0]} | {key[1]} | {key[2]:.2f} | всего | {s["n"]} | {s["fp_frames"]} | {100.0 * s["fp_frames"] / s["n"]:.1f}% |')
            for t, (n, k) in sorted(s['by'].items()):
                w(f'| {key[0]} | {key[1]} | {key[2]:.2f} | {t} | {n} | {k} | {100.0 * k / n:.1f}% |')
        w('')
    with open(os.path.join(base, 'summary.md'), 'w') as f:
        f.write('\n'.join(L) + '\n')
    print('summary:', os.path.join(base, 'summary.md'))


# ------------------------------------------------------------------ задержка

def cmd_latency(a):
    import torch
    from ultralytics import YOLO
    base = os.path.join(OUT, 'yolo_bench')
    meta, frames = load_frames('bench')
    rows = []
    for m in MODELS:
        net = YOLO(weights(m))
        for sz in IMGSZ:
            for _ in range(30):
                net.predict(frames[0], imgsz=sz, conf=0.25, classes=[0], verbose=False, device=0)
            tot, pre, inf, post = [], [], [], []
            for i in range(a.iters):
                fr = frames[i % len(frames)]
                torch.cuda.synchronize()
                t0 = time.perf_counter()
                r = net.predict(fr, imgsz=sz, conf=0.25, classes=[0], verbose=False, device=0)[0]
                torch.cuda.synchronize()
                tot.append((time.perf_counter() - t0) * 1000)
                pre.append(r.speed['preprocess'])
                inf.append(r.speed['inference'])
                post.append(r.speed['postprocess'])
            rows.append([m, sz, np.mean(tot), np.percentile(tot, 95), np.mean(pre), np.mean(inf), np.mean(post)])
            print(m, sz, f'{np.mean(tot):.1f} ms', flush=True)
        del net
    with open(os.path.join(base, 'latency.csv'), 'w', newline='') as f:
        wr = csv.writer(f)
        wr.writerow(['model', 'imgsz', 'total_ms_mean', 'total_ms_p95', 'pre_ms', 'infer_ms', 'post_ms'])
        for r in rows:
            wr.writerow([r[0], r[1]] + [f'{v:.2f}' for v in r[2:]])
    with open(os.path.join(base, 'latency.md'), 'w') as f:
        f.write('| модель | imgsz | всего мс (среднее) | p95 | pre | infer | post |\n|---|---|---|---|---|---|---|\n')
        for r in rows:
            f.write(f'| {r[0]} | {r[1]} | ' + ' | '.join(f'{v:.1f}' for v in r[2:]) + ' |\n')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest='cmd', required=True)
    sp.add_parser('sanity').set_defaults(f=cmd_sanity)
    p = sp.add_parser('capture-bench')
    p.add_argument('--frames', type=int, default=12)
    p.add_argument('--seed', type=int, default=0)
    p.set_defaults(f=cmd_capture_bench)
    p = sp.add_parser('capture-scene')
    p.add_argument('--frames', type=int, default=60, help='кадров на seed')
    p.add_argument('--seeds', default='1,2,3,4', help='seed миров через запятую')
    p.set_defaults(f=cmd_capture_scene)
    sp.add_parser('eval').set_defaults(f=cmd_eval)
    p = sp.add_parser('latency')
    p.add_argument('--iters', type=int, default=200)
    p.set_defaults(f=cmd_latency)
    a = ap.parse_args()
    a.f(a)


if __name__ == '__main__':
    main()

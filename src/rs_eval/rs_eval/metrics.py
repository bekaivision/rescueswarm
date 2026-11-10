"""Чистые функции метрик для оценки локализации (ATE, RPE) и карты. Только numpy (scipy по возможности).

Используются скриптами scripts/ate_eval.py и scripts/map_eval.py (на хосте) и тестами.
Позы - массивы (N, 3) = x, y, yaw; время - массив (N,) по возрастанию.
"""
import numpy as np


def wrap(a):
    """Угол в (-pi, pi]."""
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


# --- траектории ---------------------------------------------------------------

def interp_poses(t_src, poses, t_query):
    """Линейная интерполяция поз (x, y, yaw) в моменты t_query. Вне диапазона t_src - NaN."""
    t_src = np.asarray(t_src, dtype=float)
    poses = np.asarray(poses, dtype=float)
    out = np.full((len(t_query), 3), np.nan)
    if len(t_src) < 2:
        return out
    ok = (t_query >= t_src[0]) & (t_query <= t_src[-1])
    tq = np.asarray(t_query)[ok]
    out[ok, 0] = np.interp(tq, t_src, poses[:, 0])
    out[ok, 1] = np.interp(tq, t_src, poses[:, 1])
    out[ok, 2] = np.interp(tq, t_src, np.unwrap(poses[:, 2]))
    out[ok, 2] = wrap(out[ok, 2])
    return out


def align_se2(est_xy, gt_xy):
    """Umeyama без масштаба в 2D: (R, t) так, что R @ est + t ~ gt (МНК)."""
    est_xy = np.asarray(est_xy, dtype=float)
    gt_xy = np.asarray(gt_xy, dtype=float)
    me, mg = est_xy.mean(axis=0), gt_xy.mean(axis=0)
    H = (est_xy - me).T @ (gt_xy - mg)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, d]) @ U.T
    return R, mg - R @ me


def apply_se2(R, t, poses):
    """Применить (R, t) к позам (N, 3)."""
    out = np.array(poses, dtype=float)
    out[:, :2] = out[:, :2] @ R.T + t
    out[:, 2] = wrap(out[:, 2] + np.arctan2(R[1, 0], R[0, 0]))
    return out


def ate(est, gt):
    """ATE по позициям: словарь rmse/mean/max/final (м) и ряд ошибок. est, gt - (N, 3) на одних временах."""
    e = np.hypot(est[:, 0] - gt[:, 0], est[:, 1] - gt[:, 1])
    if len(e) == 0:
        return {'n': 0, 'rmse': float('nan'), 'mean': float('nan'), 'max': float('nan'),
                'final': float('nan')}, e
    return {'n': int(len(e)), 'rmse': float(np.sqrt(np.mean(e ** 2))), 'mean': float(e.mean()),
            'max': float(e.max()), 'final': float(e[-1])}, e


def path_length(gt):
    seg = np.hypot(np.diff(gt[:, 0]), np.diff(gt[:, 1]))
    return np.concatenate([[0.0], np.cumsum(seg)])


def _rel(a, b):
    """Поза b в системе a: (dx, dy, dyaw); a, b - (N, 3)."""
    c, s = np.cos(a[:, 2]), np.sin(a[:, 2])
    dx, dy = b[:, 0] - a[:, 0], b[:, 1] - a[:, 1]
    return np.stack([c * dx + s * dy, -s * dx + c * dy, wrap(b[:, 2] - a[:, 2])], axis=1)


def rpe(est, gt, delta):
    """RPE на отрезках истинного пути длиной delta м: ошибка относительного смещения.

    Возвращает словарь trans_rmse/trans_mean (м), rot_rmse/rot_mean (град), n (число отрезков)
    или None, если путь короче delta.
    """
    s = path_length(gt)
    if len(s) < 2 or s[-1] < delta:
        return None
    i = np.arange(len(s))
    j = np.searchsorted(s, s + delta, side='left')
    ok = j < len(s)
    i, j = i[ok], j[ok]
    if len(i) == 0:
        return None
    rg = _rel(gt[i], gt[j])
    re_ = _rel(est[i], est[j])
    et = np.hypot(re_[:, 0] - rg[:, 0], re_[:, 1] - rg[:, 1])
    er = np.abs(wrap(re_[:, 2] - rg[:, 2]))
    return {'n': int(len(i)), 'trans_rmse': float(np.sqrt(np.mean(et ** 2))), 'trans_mean': float(et.mean()),
            'rot_rmse_deg': float(np.degrees(np.sqrt(np.mean(er ** 2)))), 'rot_mean_deg': float(np.degrees(er.mean()))}


# --- карта --------------------------------------------------------------------

def edt(mask_obstacle):
    """Расстояние (в клетках) от каждой клетки до ближайшей клетки mask_obstacle (True)."""
    m = np.asarray(mask_obstacle, dtype=bool)
    if not m.any():
        return np.full(m.shape, np.inf)
    try:
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            from scipy.ndimage import distance_transform_edt
        return distance_transform_edt(~m)
    except ImportError:
        return chamfer_edt(m)


def chamfer_edt(m):
    """Запасной вариант без scipy: хемфер 3-4 (погрешность до ~8%)."""
    h, w = m.shape
    big = 1e9
    d = np.where(m, 0.0, big)
    a, b = 1.0, 1.4142
    for y in range(h):
        row = d[y]
        if y > 0:
            prev = d[y - 1]
            row[:] = np.minimum(row, prev + a)
            row[1:] = np.minimum(row[1:], prev[:-1] + b)
            row[:-1] = np.minimum(row[:-1], prev[1:] + b)
        for x in range(1, w):
            if row[x - 1] + a < row[x]:
                row[x] = row[x - 1] + a
    for y in range(h - 1, -1, -1):
        row = d[y]
        if y < h - 1:
            nxt = d[y + 1]
            row[:] = np.minimum(row, nxt + a)
            row[1:] = np.minimum(row[1:], nxt[:-1] + b)
            row[:-1] = np.minimum(row[:-1], nxt[1:] + b)
        for x in range(w - 2, -1, -1):
            if row[x + 1] + a < row[x]:
                row[x] = row[x + 1] + a
    return d


def map_metrics(est, gt_occ, res, occ_thresh=50):
    """Качество карты занятости est (int8: -1 неизвестно, 0 свободно, 100 занято) относительно gt_occ (bool).

    Обе сетки одинаковой формы и в одной системе (строка 0 = низ). Метрики:
      precision_tol{1,2}  доля занятых клеток карты, у которых в радиусе k клеток есть истинное препятствие
      recall_tol{1,2}     доля истинных препятствий, у которых в радиусе k клеток есть занятая клетка карты
      recall_seen_tol{1,2} то же, но только для препятствий рядом (<= 2 клеток) с известной клеткой карты
      wrong_free_cells    клеток внутри истинных препятствий, которые карта считает свободными
      wrong_free_tol1     из них те, рядом с которыми (<= 1 клетки) нет занятой клетки карты
      dist_mean_m / dist_median_m / dist_p95_m  расстояние от занятых клеток карты до ближайшего препятствия
    """
    est = np.asarray(est)
    if est.shape != gt_occ.shape:
        raise ValueError(f'форма карты {est.shape} != формы эталона {gt_occ.shape}')
    occ = est >= occ_thresh
    known = est >= 0
    d_true = edt(gt_occ)
    d_est = edt(occ)
    d_known = edt(known)
    out = {'n_map_occupied': int(occ.sum()), 'n_true_occupied': int(gt_occ.sum()),
           'n_true_occupied_seen': int((gt_occ & (d_known <= 2)).sum())}
    seen = gt_occ & (d_known <= 2)
    for k in (1, 2):
        out[f'precision_tol{k}'] = float((d_true[occ] <= k).mean()) if occ.any() else float('nan')
        out[f'recall_tol{k}'] = float((d_est[gt_occ] <= k).mean()) if gt_occ.any() else float('nan')
        out[f'recall_seen_tol{k}'] = float((d_est[seen] <= k).mean()) if seen.any() else float('nan')
    wf = gt_occ & (est == 0)
    out['wrong_free_cells'] = int(wf.sum())
    out['wrong_free_tol1'] = int((wf & (d_est > 1)).sum())
    if occ.any():
        dm = d_true[occ] * res
        out.update(dist_mean_m=float(dm.mean()), dist_median_m=float(np.median(dm)),
                   dist_p95_m=float(np.percentile(dm, 95)))
    else:
        out.update(dist_mean_m=float('nan'), dist_median_m=float('nan'), dist_p95_m=float('nan'))
    return out

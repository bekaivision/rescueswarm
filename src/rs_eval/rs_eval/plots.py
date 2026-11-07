"""Графики и картинки оценки (matplotlib, бэкенд Agg)."""
import csv

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

COLORS = ['#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4', '#46f0f0']


def read_csv(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r['sim_time_s']) for r in rows])
    c = np.array([float(r['coverage']) for r in rows])
    return t, c, rows


def time_to(t, c, level):
    idx = np.flatnonzero(c >= level)
    return float(t[idx[0]]) if len(idx) else None


def plot_coverage(csv_paths, out_png, labels=None, target=0.9):
    fig, ax = plt.subplots(figsize=(7, 4), dpi=120)
    for i, p in enumerate(csv_paths):
        t, c, _ = read_csv(p)
        lab = labels[i] if labels else p.split('/')[-2] if '/' in p else p
        ax.plot(t, 100 * c, color=COLORS[i % len(COLORS)], label=lab, lw=1.8)
    ax.axhline(100 * target, color='gray', ls='--', lw=1)
    ax.set_xlabel('время симуляции, с')
    ax.set_ylabel('покрытие маски, %')
    ax.set_ylim(0, 100)
    ax.grid(alpha=0.3)
    ax.legend(loc='lower right')
    fig.tight_layout()
    fig.savefig(out_png)
    plt.close(fig)


def save_map_png(data, res, ox, oy, mask, trails, out_png, title=''):
    """data: int8 (h, w); mask: bool (h, w) или None; trails: {robot: [(x, y), ...]}."""
    h, w = data.shape
    img = np.full((h, w, 3), 0.55)           # неизвестно: серый
    img[data == 0] = (1.0, 1.0, 1.0)
    img[data >= 65] = (0.0, 0.0, 0.0)
    if mask is not None:
        miss = mask & (data < 0)             # в маске, но не исследовано: красноватый
        img[miss] = (0.95, 0.75, 0.75)
    extent = (ox, ox + w * res, oy, oy + h * res)
    fig, ax = plt.subplots(figsize=(9, 7.8), dpi=130)
    ax.imshow(img, origin='lower', extent=extent, interpolation='nearest')
    if mask is not None:
        ax.contour(np.linspace(ox, ox + w * res, w), np.linspace(oy, oy + h * res, h),
                   mask.astype(float), levels=[0.5], colors=['#1f77b4'], linewidths=0.8)
    for i, (r, pts) in enumerate(sorted(trails.items())):
        if pts:
            a = np.array(pts)
            ax.plot(a[:, 0], a[:, 1], color=COLORS[i % len(COLORS)], lw=1.6, label=r)
            ax.plot(a[0, 0], a[0, 1], 'o', color=COLORS[i % len(COLORS)], ms=5)
            ax.plot(a[-1, 0], a[-1, 1], '*', color=COLORS[i % len(COLORS)], ms=10)
    ax.set_xlabel('x, м')
    ax.set_ylabel('y, м')
    ax.set_title(title)
    ax.legend(loc='upper right', fontsize=8)
    fig.tight_layout()
    fig.savefig(out_png)
    plt.close(fig)

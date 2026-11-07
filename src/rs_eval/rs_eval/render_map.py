"""CLI: итоговая карта в PNG с траекториями из файлов прогона (final_map.npy + trajectories.csv).

  ros2 run rs_eval render_map RUN_DIR WORLD_DIR [--out map.png]
"""
import argparse
import csv
import os

import numpy as np

from rs_eval import plots
from rs_eval.coverage import load_mask


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('run_dir')
    ap.add_argument('world_dir')
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    mask, res, ox, oy = load_mask(a.world_dir)
    data = np.load(os.path.join(a.run_dir, 'final_map.npy'))
    trails = {}
    with open(os.path.join(a.run_dir, 'trajectories.csv')) as f:
        for r in csv.DictReader(f):
            trails.setdefault(r['robot'], []).append((float(r['x']), float(r['y'])))
    out = a.out or os.path.join(a.run_dir, 'final_map.png')
    plots.save_map_png(data, res, ox, oy, mask, trails, out, os.path.basename(a.run_dir.rstrip('/')))
    print(out)


if __name__ == '__main__':
    main()

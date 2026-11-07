"""CLI: график покрытия по времени из одного или нескольких CSV.

  ros2 run rs_eval plot_coverage --out cov.png run1/coverage.csv run2/coverage.csv
  ros2 run rs_eval plot_coverage --map-png map.png --map-npy run1/final_map.npy ...  (см. --help)
"""
import argparse
import os

from rs_eval import plots


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('csv', nargs='+')
    ap.add_argument('--out', default='coverage.png')
    ap.add_argument('--labels', nargs='*')
    ap.add_argument('--target', type=float, default=0.9)
    a = ap.parse_args()
    labels = a.labels or [os.path.basename(os.path.dirname(os.path.abspath(p))) for p in a.csv]
    plots.plot_coverage(a.csv, a.out, labels, a.target)
    for p, lab in zip(a.csv, labels):
        t, c, _ = plots.read_csv(p)
        marks = ' '.join(f'{int(100 * l)}%: {plots.time_to(t, c, l)}' for l in (0.5, 0.8, 0.9, 0.95))
        print(f'{lab}: итог {100 * c[-1]:.1f} % за {t[-1]:.0f} с | {marks}')


if __name__ == '__main__':
    main()

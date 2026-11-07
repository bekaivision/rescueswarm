import os

import numpy as np

from rs_eval import plots


def test_time_to_and_plot(tmp_path):
    p = tmp_path / 'coverage.csv'
    p.write_text('sim_time_s,coverage,n_known,n_mask,dist_ground_1\n'
                 '0,0.0,0,100,0\n10,0.5,50,100,5\n20,0.9,90,100,9\n')
    t, c, rows = plots.read_csv(str(p))
    assert plots.time_to(t, c, 0.5) == 10.0
    assert plots.time_to(t, c, 0.95) is None
    out = tmp_path / 'cov.png'
    plots.plot_coverage([str(p)], str(out), ['x'])
    assert os.path.getsize(out) > 1000


def test_map_png(tmp_path):
    data = -np.ones((40, 40), dtype=np.int8)
    data[10:30, 10:30] = 0
    data[10, 10:30] = 100
    mask = np.zeros((40, 40), dtype=bool)
    mask[5:35, 5:35] = True
    out = tmp_path / 'map.png'
    plots.save_map_png(data, 0.05, 0.0, 0.0, mask, {'ground_1': [(0.2, 0.2), (1.0, 1.0)]}, str(out), 't')
    assert os.path.getsize(out) > 1000

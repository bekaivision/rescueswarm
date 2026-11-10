import numpy as np

from rs_eval import metrics


def _line(n=200, step=0.05):
    x = np.arange(n) * step
    return np.stack([x, np.zeros(n), np.zeros(n)], axis=1)


def test_ate_zero_and_offset():
    g = _line()
    a, _ = metrics.ate(g.copy(), g)
    assert a['rmse'] == 0 and a['max'] == 0 and a['final'] == 0
    e = g.copy()
    e[:, 1] += 0.1
    a, _ = metrics.ate(e, g)
    assert abs(a['rmse'] - 0.1) < 1e-9 and abs(a['final'] - 0.1) < 1e-9


def test_interp_poses():
    t = np.array([0.0, 1.0, 2.0])
    p = np.array([[0, 0, 3.1], [1, 0, -3.1], [2, 0, 3.1]])
    q = metrics.interp_poses(t, p, np.array([0.5, 2.5]))
    assert abs(q[0, 0] - 0.5) < 1e-9
    assert abs(abs(q[0, 2]) - np.pi) < 0.1      # через разрыв ±pi, а не через 0
    assert np.isnan(q[1]).all()


def test_align_se2_recovers_transform():
    rng = np.random.default_rng(0)
    gt = rng.uniform(-5, 5, (50, 2))
    th = 0.4
    R0 = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    est = (gt - [1.0, 2.0]) @ R0                # est = R0^T (gt - t0)
    R, t = metrics.align_se2(est, gt)
    assert np.abs(est @ R.T + t - gt).max() < 1e-9


def test_rpe_drift():
    g = _line(400)                              # 20 м прямо
    e = g.copy()
    e[:, 0] *= 1.01                             # масштабный дрейф 1 %
    r = metrics.rpe(e, g, 1.0)
    assert abs(r['trans_mean'] - 0.01) < 1e-3
    assert metrics.rpe(e, g, 50.0) is None
    z = metrics.rpe(g.copy(), g, 5.0)
    assert z['trans_rmse'] < 1e-9 and z['rot_rmse_deg'] < 1e-9


def test_edt_chamfer_close_to_exact():
    m = np.zeros((40, 50), dtype=bool)
    m[10, 20] = True
    d = metrics.chamfer_edt(m)
    exact = np.hypot(*np.meshgrid(np.arange(40) - 10, np.arange(50) - 20, indexing='ij'))
    assert np.abs(d - exact).max() < 0.1 * exact.max()
    assert abs(metrics.edt(m)[10, 25] - 5.0) < 0.5


def test_map_metrics_perfect_and_shifted():
    gt = np.zeros((40, 40), dtype=bool)
    gt[10, 5:35] = True
    est = np.zeros((40, 40), dtype=np.int8)
    est[gt] = 100
    m = metrics.map_metrics(est, gt, 0.05)
    assert m['precision_tol1'] == 1.0 and m['recall_tol1'] == 1.0 and m['wrong_free_cells'] == 0
    assert m['dist_mean_m'] == 0
    sh = np.zeros_like(est)
    sh[13, 5:35] = 100                          # сдвиг на 3 клетки
    m = metrics.map_metrics(sh, gt, 0.05)
    assert m['precision_tol2'] == 0.0 and abs(m['dist_mean_m'] - 0.15) < 1e-6
    assert m['wrong_free_cells'] == 30

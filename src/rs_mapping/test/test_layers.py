"""Квантование слоёв и слияние слоёв карты (Фаза 6)."""
import numpy as np

from rs_mapping import layers as L
from rs_mapping.air_update import fuse
from rs_mapping.grid_update import GridUpdater


def test_quantize():
    lo = np.array([0.0, 0.5, -2.0, 3.5, 9.0, 1.0], dtype=np.float32)
    seen = np.array([True, True, True, True, True, False])
    q = L.quantize(lo, seen)
    assert q.dtype == np.int8
    assert q.tolist() == [0, 10, -40, 70, 100, -128]


def test_merge_sum_and_clip():
    a = np.array([40, -128, 70, -128], dtype=np.int8)   # 2.0, -, 3.5, -
    b = np.array([40, 20, 70, -128], dtype=np.int8)     # 2.0, 1.0, 3.5, -
    Lm, seen = L.merge_layers([a, b])
    assert seen.tolist() == [True, True, True, False]
    assert np.allclose(Lm[:3], [3.5, 1.0, 3.5])         # 4.0 -> 3.5, 7.0 -> 3.5
    assert L.to_grid(Lm, seen).tolist() == [100, 100, 100, -1]


def test_merge_free_and_negative_sum():
    a = np.array([-40, 10], dtype=np.int8)
    b = np.array([20, -20], dtype=np.int8)
    g = L.merge_grid([a, b])
    assert g.tolist() == [0, 0]   # -1.0, -0.5 -> свободно
    assert L.merge_grid([]) is None


def test_one_source_matches_gridupdater():
    up = GridUpdater(4, 3, 0.1, 0.0, 0.0)
    up.logodds[:] = np.linspace(-2, 3.5, 12)
    up.seen[:] = True
    up.seen[3] = False
    q = L.quantize(up.logodds, up.seen)
    # квантование шагом 0.05 не меняет порога 0.4 (0.4 = 8 * 0.05), кроме клеток точно у порога
    ref = up.to_grid_data()
    got = L.merge_grid([q])
    assert (ref == got).all()


def test_fuse_grids():
    g = np.array([-1, 0, 100, -1], dtype=np.int8)
    a = np.array([100, 100, 0, -1], dtype=np.int8)
    assert L.fuse_grids(g, a).tolist() == [100, 0, 100, -1]
    assert L.fuse_grids(g, None).tolist() == g.tolist()
    assert L.fuse_grids(None, a).tolist() == a.tolist()
    assert L.fuse_grids(None, None) is None


def test_fuse_matches_air_update_fuse():
    class G:
        def __init__(self, seen, d):
            self.seen, self._d = seen, d

        def to_grid_data(self):
            return self._d
    g = G(np.array([True, False, True]), np.array([0, -1, 100], dtype=np.int8))
    a = G(np.array([True, True, False]), np.array([100, 0, -1], dtype=np.int8))
    assert (fuse(g, a) == L.fuse_grids(g._d, a._d)).all()

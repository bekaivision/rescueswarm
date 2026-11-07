import math

import numpy as np

from rs_mapping.footprint_scan import make_angles, ray_rect_ranges


def test_no_others_all_inf():
    r = ray_rect_ranges(make_angles(72), (0, 0, 0), [])
    assert np.isinf(r).all()


def test_head_on_range():
    # другой робот в 2 м впереди, смотрит на нас: ближайшая грань на 2 - 0.25 - pad
    a = make_angles(720)
    r = ray_rect_ranges(a, (0, 0, 0), [(2.0, 0.0, math.pi)], pad=0.0)
    i = int(np.argmin(np.abs(a)))
    assert abs(r[i] - 1.75) < 1e-6
    assert np.isinf(r[int(np.argmin(np.abs(a - math.pi)))])    # позади ничего нет


def test_yaw_of_me_rotates_rays():
    # я повёрнут на 90 градусов, другой на оси x мира: он виден справа (угол -90 в моём кадре)
    a = make_angles(720)
    r = ray_rect_ranges(a, (0, 0, math.pi / 2), [(2.0, 0.0, 0.0)], pad=0.0)
    i = int(np.argmin(np.abs(a + math.pi / 2)))
    assert abs(r[i] - 1.75) < 1e-6      # луч упирается в торец (полудлина 0.25)
    assert np.isfinite(r).sum() > 5


def test_out_of_range_is_inf():
    r = ray_rect_ranges(make_angles(720), (0, 0, 0), [(20.0, 0, 0)], max_range=8.0)
    assert np.isinf(r).all()

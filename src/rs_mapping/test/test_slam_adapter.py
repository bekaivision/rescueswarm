import math

import numpy as np

from rs_mapping.slam_adapter import from_xyyaw, world_odom


def test_world_odom_z_is_base_height():
    # колёсная одометрия плоская (z = 0): world -> odom обязан поднять базу на высоту base_link над полом
    T_ob = from_xyyaw(1.0, 0.5, 0.3)
    T_wb = from_xyyaw(3.0, -2.0, 1.0)
    T_wb[2, 3] = 0.01          # z от SLAM не должен просочиться
    T = world_odom(T_wb, T_ob, 0.08)
    assert abs(T[2, 3] - 0.08) < 1e-12
    T_world_base = T @ T_ob    # база в world: x, y, курс как у SLAM, z = 0.08
    assert np.allclose(T_world_base[:2, 3], T_wb[:2, 3])
    assert math.isclose(math.atan2(T_world_base[1, 0], T_world_base[0, 0]), 1.0, abs_tol=1e-9)
    assert abs(T_world_base[2, 3] - 0.08) < 1e-12

"""Мелкая геометрия: TransformStamped -> матрица 4x4, применение к точкам."""
import numpy as np


def tf_matrix(t):
    """geometry_msgs/TransformStamped -> numpy 4x4 (child -> parent)."""
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 0 else 0.0
    m = np.eye(4)
    m[0, 0] = 1 - s * (y * y + z * z)
    m[0, 1] = s * (x * y - z * w)
    m[0, 2] = s * (x * z + y * w)
    m[1, 0] = s * (x * y + z * w)
    m[1, 1] = 1 - s * (x * x + z * z)
    m[1, 2] = s * (y * z - x * w)
    m[2, 0] = s * (x * z - y * w)
    m[2, 1] = s * (y * z + x * w)
    m[2, 2] = 1 - s * (x * x + y * y)
    tr = t.transform.translation
    m[:3, 3] = (tr.x, tr.y, tr.z)
    return m


def apply(m, pts):
    """pts (N, 3) -> (N, 3)."""
    return pts @ m[:3, :3].T + m[:3, 3]


def invert(m):
    r = m[:3, :3]
    out = np.eye(4)
    out[:3, :3] = r.T
    out[:3, 3] = -r.T @ m[:3, 3]
    return out

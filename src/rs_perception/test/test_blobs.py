from rs_perception import blobs


def test_lidar_consistent_ok_and_foreground_object():
    fy, cam_z = 138.7, 0.14
    assert blobs.lidar_consistent(3.0, 3.3, fy, cam_z)          # в пределах допуска
    assert not blobs.lidar_consistent(1.4, 3.4, fy, cam_z)      # ближний предмет (огонь) вместо человека
    assert not blobs.lidar_consistent(3.4, 1.4, fy, cam_z)
    assert blobs.lidar_consistent(1.4, None, fy, cam_z)         # нет оценки по полу: не проверяем


def test_ground_range_none_above_horizon():
    from rs_perception import blobs
    fx = fy = 138.6
    cx, cy = 79.5, 59.5
    # низ пятна выше центра кадра (камера горизонтальна): луч не пересекает пол
    b = dict(u0=70, u1=80, v0=50, v1=58)
    assert blobs.ground_range(b, fx, fy, cx, cy, 0.22, 1e9) is None
    b = dict(u0=70, u1=80, v0=60, v1=75)
    assert blobs.ground_range(b, fx, fy, cx, cy, 0.22, 1e9) is not None

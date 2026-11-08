from rs_perception import blobs


def test_lidar_consistent_ok_and_foreground_object():
    fy, cam_z = 138.7, 0.14
    assert blobs.lidar_consistent(3.0, 3.3, fy, cam_z)          # в пределах допуска
    assert not blobs.lidar_consistent(1.4, 3.4, fy, cam_z)      # ближний предмет (огонь) вместо человека
    assert not blobs.lidar_consistent(3.4, 1.4, fy, cam_z)
    assert blobs.lidar_consistent(1.4, None, fy, cam_z)         # нет оценки по полу: не проверяем

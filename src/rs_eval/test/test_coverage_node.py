"""Узел coverage с дронами и без: формат файлов, слои покрытия, условие завершения (нужен rclpy)."""
import csv
import os

import numpy as np
import pytest
import yaml
from PIL import Image

rclpy = pytest.importorskip('rclpy')
from nav_msgs.msg import OccupancyGrid, Odometry  # noqa: E402
from rs_interfaces.msg import ExplorationBid  # noqa: E402

from rs_eval import coverage  # noqa: E402

W, H = 20, 10


@pytest.fixture(scope='module', autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _mask_dir(tmp_path):
    d = tmp_path / 'world'
    d.mkdir()
    Image.fromarray(np.full((H, W), 255, dtype=np.uint8)).save(d / 'mask.pgm')
    (d / 'coverage_mask.yaml').write_text(yaml.safe_dump({'image': 'mask.pgm', 'resolution': 0.1, 'origin': [0.0, 0.0, 0.0]}))
    return str(d)


def _make(tmp_path, air=None):
    ov = [rclpy.Parameter('mask_dir', value=_mask_dir(tmp_path)),
          rclpy.Parameter('robots', value=['ground_1']),
          rclpy.Parameter('out_dir', value=str(tmp_path / 'out')),
          rclpy.Parameter('exit_on_done', value=False),
          rclpy.Parameter('target', value=2.0)]
    if air:
        ov.append(rclpy.Parameter('air_robots', value=air))
    orig = rclpy.node.Node.__init__

    def init(self, name, **kw):
        orig(self, name, parameter_overrides=ov, **kw)
    coverage.Node.__init__ = init
    try:
        return coverage.Coverage()
    finally:
        coverage.Node.__init__ = orig


def _grid(known_cols):
    m = OccupancyGrid()
    m.info.width, m.info.height, m.info.resolution = W, H, 0.1
    d = -np.ones((H, W), dtype=np.int8)
    d[:, :known_cols] = 0
    m.data = d.flatten().tolist()
    return m


def _bid(rid, done):
    b = ExplorationBid()
    b.robot_id, b.done = rid, done
    return b


def _odom(x, t):
    o = Odometry()
    o.header.stamp.sec = int(t)
    o.pose.pose.position.x = x
    o.pose.pose.orientation.w = 1.0
    return o


def _rows(path):
    return open(path).read().splitlines()


def test_no_air_format_unchanged(tmp_path):
    n = _make(tmp_path)
    n._on_map(_grid(10))
    n._tick()
    n._save()
    out = tmp_path / 'out'
    assert _rows(out / 'coverage.csv')[0] == 'sim_time_s,coverage,n_known,n_mask,dist_ground_1'
    assert len(_rows(out / 'coverage.csv')[1].split(',')) == 5
    assert _rows(out / 'pose_log.csv')[0] == 't,robot,source,x,y,yaw'
    assert n.air == []
    n.destroy_node()


def test_air_columns_pose_log_and_completion(tmp_path):
    n = _make(tmp_path, air=['drone_1'])
    t = [100.0]
    n._now = lambda: t[0]
    n._on_map(_grid(10))
    n._on_layer('ground', _grid(5))
    n._on_layer('air', _grid(15))
    n._log_pose('drone_1', 'pose', _odom(1.0, 1))
    n._log_pose('drone_1', 'gt', _odom(1.1, 1))
    n._tick()
    rows = list(csv.DictReader(open(tmp_path / 'out' / 'coverage.csv')))
    assert list(rows[0])[-2:] == ['cov_ground', 'cov_air']
    assert float(rows[0]['cov_ground']) == 0.25 and float(rows[0]['cov_air']) == 0.75
    n.pose_log.flush()
    log = _rows(tmp_path / 'out' / 'pose_log.csv')
    assert any(',drone_1,pose,1.0000,' in r for r in log) and any(',drone_1,gt,1.1000,' in r for r in log)
    # наземный done без воздушного: не завершено
    n._on_bid(_bid('ground_1', True))
    for _ in range(12):
        t[0] += 1.0
        n._on_bid(_bid('ground_1', True))
        n._tick()
    assert not n.finished
    # воздушный done: ждём done_hold (5 с)
    n._on_air_bid(_bid('drone_1', True))
    t[0] += 1.0
    n._on_bid(_bid('ground_1', True))
    n._on_air_bid(_bid('drone_1', True))
    n._tick()
    assert not n.finished
    for _ in range(4):
        t[0] += 1.0
        n._on_bid(_bid('ground_1', True))
        n._on_air_bid(_bid('drone_1', True))
        n._tick()
    assert not n.finished
    t[0] += 1.5
    n._on_bid(_bid('ground_1', True))
    n._on_air_bid(_bid('drone_1', True))
    n._tick()
    assert n.finished and 'done' in n.finish_reason
    n.destroy_node()


def test_air_done_dropped_resets_settle(tmp_path):
    n = _make(tmp_path, air=['drone_1'])
    t = [0.0]
    n._now = lambda: t[0]
    n._on_map(_grid(10))
    for _ in range(4):
        t[0] += 1.0
        n._on_bid(_bid('ground_1', True))
        n._on_air_bid(_bid('drone_1', True))
        n._tick()
    t[0] += 1.0
    n._on_bid(_bid('ground_1', True))
    n._on_air_bid(_bid('drone_1', False))   # дрон снова работает
    n._tick()
    assert n.done_since is None and not n.finished
    n.destroy_node()

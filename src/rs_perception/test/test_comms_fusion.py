"""people_fusion: пакет ObsBatch даёт те же отметки, что прямые топики (нужен rclpy)."""
import json

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Pose, PoseArray  # noqa: E402
from rs_interfaces.msg import ObsBatch, RgbDetection, ThermalDetection  # noqa: E402

from rs_perception import people_fusion as pf  # noqa: E402


@pytest.fixture(scope='module', autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make(out, obs_topic=''):
    ov = [rclpy.Parameter('out_dir', value=str(out)), rclpy.Parameter('robots', value=['ground_1'])]
    if obs_topic:
        ov.append(rclpy.Parameter('obs_topic', value=obs_topic))
    orig = rclpy.node.Node.__init__

    def init(self, name, **kw):
        orig(self, name, parameter_overrides=ov, **kw)
    pf.Node.__init__ = init
    try:
        return pf.PeopleFusion()
    finally:
        pf.Node.__init__ = orig


def _det(t, x=1.0, y=2.0):
    d = ThermalDetection()
    d.header.stamp.sec, d.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
    d.robot, d.position.x, d.position.y = 'ground_1', x, y
    d.range_m, d.mean_temp_k, d.max_temp_k = 2.0, 310.0, 312.0
    d.height_m, d.length_m, d.lidar_pad = 0.2, 1.5, 0
    return d


def _rgb(t):
    d = RgbDetection()
    d.header.stamp.sec = int(t)
    d.robot, d.bearing_rad, d.half_width_rad, d.score = 'ground_1', 0.0, 0.1, 0.9
    return d


def _summary(node):
    return sorted((m.id, m.status, round(m.x, 3), round(m.y, 3), m.n_obs, m.rgb_hits)
                  for m in node.fusion.marks)


def test_batch_equals_direct(tmp_path):
    old = _make(tmp_path / 'old')
    new = _make(tmp_path / 'new', obs_topic='/ground_1/comms/obs')
    # поза: старый путь берёт её из /fleet/poses, новый - из вида в пакете; time вида 0.5 (до детекций)
    pa = PoseArray()
    p = Pose()
    p.position.x, p.position.y, p.orientation.w = 0.0, 0.0, 1.0
    pa.poses = [p]
    old._on_poses(pa)
    old.poses['ground_1'] = (0.0, 0.0, 0.0)
    dets = [_det(1.0 + i, y=2.0 + 0.01 * i) for i in range(3)]
    rgbs = [_rgb(1.5)]
    for d in dets:
        old._on_det(d)
    old._on_rgb(rgbs[0])
    b = ObsBatch()
    b.robot = 'ground_1'
    b.detections = dets
    b.rgb = rgbs
    b.rgb_pose = [0.0, 0.0, 0.0]
    b.view_t = [0.5]
    b.view_pose = [0.0, 0.0, 0.0]
    new._on_batch(b)
    assert _summary(old) == _summary(new) and _summary(new)
    assert new.poses['ground_1'] == (0.0, 0.0, 0.0)
    for n in (old, new):
        n.det_log.flush()
        n.rgb_log.flush()
    lo = [json.loads(s) for s in open(tmp_path / 'old' / 'detections.jsonl')]
    ln = [json.loads(s) for s in open(tmp_path / 'new' / 'detections.jsonl')]
    assert all('rx' not in r for r in lo) and all('rx' in r for r in ln)
    assert [{k: v for k, v in r.items() if k != 'rx'} for r in ln] == lo
    rn = [json.loads(s) for s in open(tmp_path / 'new' / 'rgb_detections.jsonl')]
    assert 'rx' in rn[0]


def test_empty_robots_gives_clear_error(tmp_path):
    ov = [rclpy.Parameter('out_dir', value=str(tmp_path))]
    orig = rclpy.node.Node.__init__

    def init(self, name, **kw):
        orig(self, name, parameter_overrides=ov, **kw)
    pf.Node.__init__ = init
    try:
        with pytest.raises(ValueError, match='robots пуст'):
            pf.PeopleFusion()
    finally:
        pf.Node.__init__ = orig

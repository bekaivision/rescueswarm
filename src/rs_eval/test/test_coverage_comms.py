"""Узел coverage в режиме связи: личные топики bids, столбец coverage_collective (нужен rclpy)."""
import pytest

rclpy = pytest.importorskip('rclpy')
from rs_eval import coverage  # noqa: E402
from test_coverage_node import _bid, _grid, _make as _make_plain, _rows  # noqa: E402,F401
import test_coverage_node as base  # noqa: E402


@pytest.fixture(scope='module', autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make(tmp_path):
    ov = [rclpy.Parameter('mask_dir', value=base._mask_dir(tmp_path)),
          rclpy.Parameter('robots', value=['ground_1']),
          rclpy.Parameter('out_dir', value=str(tmp_path / 'out')),
          rclpy.Parameter('exit_on_done', value=False),
          rclpy.Parameter('comms', value=True),
          rclpy.Parameter('done_hold', value=0.0),
          rclpy.Parameter('target', value=2.0)]
    orig = rclpy.node.Node.__init__

    def init(self, name, **kw):
        orig(self, name, parameter_overrides=ov, **kw)
    coverage.Node.__init__ = init
    try:
        return coverage.Coverage()
    finally:
        coverage.Node.__init__ = orig


def _have(node, m, o):
    from rs_interfaces.msg import CommsBeacon
    b = CommsBeacon()
    b.node = node
    b.map_sources = list(m)
    b.map_versions = list(m.values())
    b.obs_sources = list(o)
    b.obs_seqs = list(o.values())
    return b


def test_collective_column_and_sync_wait(tmp_path):
    n = _make(tmp_path)
    n._on_map(_grid(5))
    n._on_collective(_grid(15))
    n._on_have(_have('ground_1', {'ground_1': 40}, {'ground_1': 20}))
    n._on_have(_have('base', {'ground_1': 30}, {'ground_1': 10}))
    n._on_bid(_bid('ground_1', True))
    n._tick()
    # база отстаёт от ground_1 по версиям: ждём
    assert not n.finished and n.sync_start is not None
    assert n.sync_target == {'ground_1': (40, 20)}
    n._on_have(_have('ground_1', {'ground_1': 45}, {'ground_1': 22}))   # цель ожидания не двигается
    n._on_have(_have('base', {'ground_1': 40}, {'ground_1': 20}))
    n._on_map(_grid(15))
    n._tick()
    assert n.finished and 'done' in n.finish_reason
    rows = _rows(tmp_path / 'out' / 'coverage.csv')
    assert rows[0].split(',')[-1] == 'coverage_collective'
    assert rows[1].split(',')[-1] == '0.75000'
    assert (tmp_path / 'out' / 'final_map_collective.npy').exists()


def test_version_sync_ok():
    t = {'a': (10, 5), 'b': (3, 0)}
    assert coverage.version_sync_ok(t, {'a': 10, 'b': 3}, {'a': 5})
    assert not coverage.version_sync_ok(t, {'a': 9, 'b': 3}, {'a': 5})
    assert not coverage.version_sync_ok(t, {'a': 10, 'b': 3}, {'a': 4})
    assert coverage.version_sync_ok({}, {}, {})

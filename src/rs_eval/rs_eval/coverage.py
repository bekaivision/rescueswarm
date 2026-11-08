"""Узел coverage: покрытие карты внутри маски, CSV, флаг завершения, итоговые картинки.

Параметры: mask_dir (каталог мира с coverage_mask.pgm/.yaml), robots, out_dir, target (0.95),
max_time (сим. секунды, 0 = без лимита), exit_on_done (завершить процесс после записи).
"""
import csv
import math
import os

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid, Odometry
from PIL import Image
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rs_interfaces.msg import ExplorationBid
from std_msgs.msg import Float32

from rs_eval import plots


def load_mask(mask_dir):
    """Маска (h, w) bool, низ карты = строка 0, как в OccupancyGrid; плюс origin и resolution."""
    with open(os.path.join(mask_dir, 'coverage_mask.yaml')) as f:
        meta = yaml.safe_load(f)
    img = np.array(Image.open(os.path.join(mask_dir, meta['image'])))
    mask = img[::-1] > 127
    return mask, meta['resolution'], meta['origin'][0], meta['origin'][1]


class Coverage(Node):
    def __init__(self):
        super().__init__('coverage')
        P = self.declare_parameter
        P('mask_dir', '')
        P('robots', ['ground_1', 'ground_2', 'ground_3'])
        P('out_dir', '/tmp/rs_coverage')
        P('target', 0.95)
        P('max_time', 0.0)
        P('exit_on_done', True)
        P('search_wait', False)   # не завершать по покрытию, пока есть открытые кандидаты-люди (Фаза 3)
        P('search_hold', 3.0)     # сек. сим. времени "нет открытых кандидатов" подряд
        P('done_hold', 5.0)   # сек. сим. времени, которые все explorer должны держать done
        self.robots = list(self.get_parameter('robots').value)
        self.out_dir = self.get_parameter('out_dir').value
        os.makedirs(self.out_dir, exist_ok=True)
        self.mask, self.res, self.ox, self.oy = load_mask(self.get_parameter('mask_dir').value)
        self.n_mask = int(self.mask.sum())
        self.target = float(self.get_parameter('target').value)
        self.max_time = float(self.get_parameter('max_time').value)
        self.map = None
        self.cov = 0.0
        self.n_known = 0
        self.t0 = None
        self.dist = {r: 0.0 for r in self.robots}
        self.last = {}
        self.trails = {r: [] for r in self.robots}
        self.done_since = None
        self.bids_done = {}
        self.bids_open = {}    # robot -> (время, открытых кандидатов, отказов)
        self.search_free_since = None
        self.search_logged = False
        self.finished = False
        self.finish_reason = ''
        self.csv_path = os.path.join(self.out_dir, 'coverage.csv')
        self.f = open(self.csv_path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['sim_time_s', 'coverage', 'n_known', 'n_mask'] + [f'dist_{r}' for r in self.robots])
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, qos)
        for r in self.robots:
            self.create_subscription(Odometry, f'/{r}/odom', lambda m, r=r: self._on_odom(r, m), 10)
        self.create_subscription(ExplorationBid, '/exploration/bids', self._on_bid, 20)
        self.pub = self.create_publisher(Float32, '/coverage', 1)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(f'маска: {self.n_mask} клеток ({self.n_mask * self.res ** 2:.1f} м2), цель {self.target:.0%}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_map(self, msg):
        self.map = msg
        d = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        if d.shape != self.mask.shape:
            self.get_logger().error(f'размер карты {d.shape} != маски {self.mask.shape}', throttle_duration_sec=10.0)
            return
        self.data = d
        self.n_known = int(((d >= 0) & self.mask).sum())
        self.cov = self.n_known / max(self.n_mask, 1)

    def _on_odom(self, r, msg):
        p = msg.pose.pose.position
        if r in self.last:
            self.dist[r] += math.hypot(p.x - self.last[r][0], p.y - self.last[r][1])
        self.last[r] = (p.x, p.y)
        tr = self.trails[r]
        if not tr or math.hypot(p.x - tr[-1][0], p.y - tr[-1][1]) > 0.1:
            tr.append((p.x, p.y))

    def _on_bid(self, msg):
        self.bids_done[msg.robot_id] = (self._now(), msg.done)
        self.bids_open[msg.robot_id] = (self._now(), int(msg.open_candidates), list(msg.gave_up))

    def _tick(self):
        if self.finished:
            return
        t = self._now()
        if self.t0 is None:
            self.t0 = t
        ts = t - self.t0
        self.pub.publish(Float32(data=float(self.cov)))
        self.w.writerow([f'{ts:.1f}', f'{self.cov:.5f}', self.n_known, self.n_mask]
                        + [f'{self.dist[r]:.2f}' for r in self.robots])
        self.f.flush()
        # все explorer сообщили done и держат это
        all_done = all(r in self.bids_done and t - self.bids_done[r][0] < 5.0 and self.bids_done[r][1]
                       for r in self.robots)
        if all_done:
            self.done_since = self.done_since if self.done_since is not None else t
        else:
            self.done_since = None
        search_ok = True
        if self.get_parameter('search_wait').value:
            fresh = [v for v in self.bids_open.values() if t - v[0] < 5.0]
            n_open = max((v[1] for v in fresh), default=0)
            if n_open == 0 and fresh:
                self.search_free_since = self.search_free_since if self.search_free_since is not None else t
            else:
                self.search_free_since = None
            search_ok = (self.search_free_since is not None
                         and t - self.search_free_since >= float(self.get_parameter('search_hold').value))
        if search_ok and (self.cov >= self.target or self.done_since is not None) and \
                self.get_parameter('search_wait').value and not self.search_logged:
            self.search_logged = True
            gave = sorted({g for v in self.bids_open.values() for g in v[2]})
            self.get_logger().info(f'SEARCH COMPLETE t={ts:.1f}: открытых кандидатов нет, оставлено: {gave or "нет"}')
        if self.cov >= self.target and search_ok:
            self._finish(ts, f'покрытие {self.cov:.1%} >= {self.target:.0%}')
        elif self.done_since is not None and search_ok and \
                t - self.done_since >= float(self.get_parameter('done_hold').value):
            self._finish(ts, 'все explorer сообщили done')
        elif self.max_time > 0 and ts >= self.max_time:
            self._finish(ts, 'достигнут max_time')

    def _finish(self, ts, reason):
        self.finished = True
        self.finish_reason = reason
        self.get_logger().info(f'EXPLORATION COMPLETE t={ts:.1f} coverage={self.cov:.4f} ({reason})')
        self._save()
        if self.get_parameter('exit_on_done').value:
            raise SystemExit(0)

    def _save(self):
        self.f.flush()
        if self.map is not None and hasattr(self, 'data'):
            title = f'карта на t={self._now() - self.t0:.0f} с, покрытие {self.cov:.1%}'
            plots.save_map_png(self.data, self.res, self.ox, self.oy, self.mask, self.trails,
                               os.path.join(self.out_dir, 'final_map.png'), title)
            np.save(os.path.join(self.out_dir, 'final_map.npy'), self.data)
        with open(os.path.join(self.out_dir, 'trajectories.csv'), 'w') as f:
            f.write('robot,x,y\n')
            for r, tr in self.trails.items():
                for x, y in tr:
                    f.write(f'{r},{x:.2f},{y:.2f}\n')
        plots.plot_coverage([self.csv_path], os.path.join(self.out_dir, 'coverage.png'),
                            labels=[os.path.basename(self.out_dir.rstrip('/'))])
        with open(os.path.join(self.out_dir, 'summary.txt'), 'w') as f:
            f.write(f'reason: {self.finish_reason}\ncoverage: {self.cov:.4f}\n')
            f.write('distance: ' + ' '.join(f'{r}={d:.1f}' for r, d in self.dist.items()) + '\n')


def main():
    rclpy.init()
    node = Coverage()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if not node.finished:
            node._save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

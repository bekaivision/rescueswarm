"""Узел coverage: покрытие карты внутри маски, CSV, флаг завершения, итоговые картинки.

Параметры: mask_dir (каталог мира с coverage_mask.pgm/.yaml), robots, out_dir, target (0.95),
max_time (сим. секунды, 0 = без лимита), exit_on_done (завершить процесс после записи),
air_robots (дроны, по умолчанию пусто; Фаза 5).

Дроны (air_robots не пусто): позы /<drone>/pose и /<drone>/gt_odom пишутся в pose_log.csv в том же
формате (для scripts/ate_eval.py); в coverage.csv в КОНЕЦ добавляются столбцы cov_ground и cov_air
(покрытие слоёв /map_ground и /map_air по тому же определению); запуск завершён, только когда done
сообщили все наземные explorer (/exploration/bids) и все воздушные (/exploration/air_bids) и это держится
done_hold с. С пустым air_robots все файлы и поведение прежние.

Режим связи (Фаза 6, comms=true): флаги done берутся из личных топиков /<r>/exploration/bids и
/<d>/exploration/air_bids; подписка на /map_collective (идеальная карта, только оценка): столбец
coverage_collective последним в coverage.csv и final_map_collective.npy. Завершение "все done"
дополнительно ждёт синхронизации: база (/map) догнала коллективную карту (sync_tol) или прошло
sync_timeout с. сим. времени с момента, когда все done.

Позы: /<robot>/pose (мировая поза, которой пользуются все; в режиме gt = истинная) и /<robot>/gt_odom
(истинная, только для оценки). Файлы в out_dir:
  trajectories.csv     robot,x,y         - траектория по pose (прореженная, шаг > 0.1 м)
  gt_trajectories.csv  robot,x,y         - то же по gt_odom
  pose_log.csv         t,robot,source,x,y,yaw - журнал по времени (source = pose | gt, ~5 Гц на источник);
                       по нему scripts/ate_eval.py считает ATE
  final_map.npy/.yaml  итоговая карта (строка 0 = низ) и её origin/resolution (для scripts/map_eval.py)
"""
import csv
import math
import os

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid, Odometry
from PIL import Image
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rs_interfaces.msg import ExplorationBid
from std_msgs.msg import Float32

from rs_eval import plots


def sync_wait_decision(base_cov, coll_cov, waited, tol, timeout):
    """Ждать ли синхронизации базы с коллективной картой: (можно_завершать, причина). Чистая функция."""
    if base_cov >= coll_cov - tol:
        return True, 'synced'
    if waited >= timeout:
        return True, 'timeout'
    return False, 'waiting'


def version_sync_ok(target, base_map, base_obs):
    """База догнала источники: для каждого источника s её версия карты и номер пачки наблюдений
    не меньше, чем были у самого s в начале ожидания. target: {s: (map_have, obs_have)}. Чистая функция."""
    return all(base_map.get(s, 0) >= m and base_obs.get(s, 0) >= o for s, (m, o) in target.items())


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
        P('robots', [], ParameterDescriptor(dynamic_typing=True))
        P('out_dir', '/tmp/rs_coverage')
        P('target', 0.95)
        P('max_time', 0.0)
        P('exit_on_done', True)
        P('search_wait', False)   # не завершать по покрытию, пока есть открытые кандидаты-люди (Фаза 3)
        P('search_hold', 3.0)     # сек. сим. времени "нет открытых кандидатов" подряд
        P('done_hold', 5.0)   # сек. сим. времени, которые все explorer должны держать done
        P('comms', False)         # Фаза 6: связь через радио, done из личных топиков, карта /map_collective
        P('sync_tol', 0.005)      # допуск отставания базы от коллективной карты (доля покрытия)
        P('sync_timeout', 120.0)  # макс. ожидание синхронизации, сим. с
        # дроны; динамический тип: пустой список по умолчанию (иначе rclpy не выводит тип пустого массива)
        self.declare_parameter('air_robots', [], ParameterDescriptor(dynamic_typing=True))
        self.robots = list(self.get_parameter('robots').value or [])
        if not self.robots:
            raise RuntimeError('параметр robots пуст: список роботов задаёт launch (explore.launch.py / slam.launch.py), запасного списка нет')
        self.air = list(self.get_parameter('air_robots').value or [])
        self.comms = bool(self.get_parameter('comms').value)
        self.cov_coll = 0.0
        self.data_coll = None
        self.sync_start = None
        self.sync_logged = False
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
        self.gt_trails = {r: [] for r in self.robots}
        self.log_last = {}
        self.done_since = None
        self.bids_done = {}
        self.air_done = {}     # дрон -> (время, done)
        self.cov_layers = {'ground': 0.0, 'air': 0.0}
        self.layer_data = {}   # последние /map_ground и /map_air (для сравнения слоёв, RQ3)
        self.bids_open = {}    # robot -> (время, открытых кандидатов, отказов)
        self.search_free_since = None
        self.search_logged = False
        self.finished = False
        self.finish_reason = ''
        self.pose_log = open(os.path.join(self.out_dir, 'pose_log.csv'), 'w')
        self.pose_log.write('t,robot,source,x,y,yaw\n')
        self.csv_path = os.path.join(self.out_dir, 'coverage.csv')
        self.f = open(self.csv_path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['sim_time_s', 'coverage', 'n_known', 'n_mask'] + [f'dist_{r}' for r in self.robots]
                        + (['cov_ground', 'cov_air'] if self.air else [])
                        + (['coverage_collective'] if self.comms else []))
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, qos)
        for r in self.robots:
            self.create_subscription(Odometry, f'/{r}/pose', lambda m, r=r: self._on_odom(r, m), 10)
            self.create_subscription(Odometry, f'/{r}/gt_odom', lambda m, r=r: self._on_gt(r, m), 10)
        if self.comms:
            self.create_subscription(OccupancyGrid, '/map_collective', self._on_collective, qos)
            # что есть у каждого узла сети (агенты публишат локально; оценке это видно, роботам нет)
            from rs_interfaces.msg import CommsBeacon
            self.have = {}
            for n in list(self.robots) + list(self.air) + ['base']:
                self.create_subscription(CommsBeacon, f'/{n}/comms/have', self._on_have, 5)
            for r in self.robots:
                self.create_subscription(ExplorationBid, f'/{r}/exploration/bids', self._on_bid, 20)
        else:
            self.create_subscription(ExplorationBid, '/exploration/bids', self._on_bid, 20)
        if self.air:
            self.create_subscription(OccupancyGrid, '/map_ground', lambda m: self._on_layer('ground', m), qos)
            self.create_subscription(OccupancyGrid, '/map_air', lambda m: self._on_layer('air', m), qos)
            for r in self.air:
                self.create_subscription(Odometry, f'/{r}/pose', lambda m, r=r: self._log_pose(r, 'pose', m), 10)
                self.create_subscription(Odometry, f'/{r}/gt_odom', lambda m, r=r: self._log_pose(r, 'gt', m), 10)
            if self.comms:
                for r in self.air:
                    self.create_subscription(ExplorationBid, f'/{r}/exploration/air_bids', self._on_air_bid, 20)
            else:
                self.create_subscription(ExplorationBid, '/exploration/air_bids', self._on_air_bid, 20)
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

    def _on_have(self, b):
        self.have[b.node] = (dict(zip(b.map_sources, (int(v) for v in b.map_versions))),
                             dict(zip(b.obs_sources, (int(v) for v in b.obs_seqs))))

    def _sync_target(self):
        """Что каждый робот и дрон знает о себе сейчас: (своя версия карты, свой номер пачки)."""
        out = {}
        for s in list(self.robots) + list(self.air):
            if s in self.have:
                m, o = self.have[s]
                out[s] = (m.get(s, 0), o.get(s, 0))
        return out

    def _on_collective(self, msg):
        """Идеальная карта (все слои слиты): покрытие по той же маске, только для оценки."""
        d = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        if d.shape == self.mask.shape:
            self.data_coll = d
            self.cov_coll = int(((d >= 0) & self.mask).sum()) / max(self.n_mask, 1)

    def _on_layer(self, name, msg):
        """Покрытие слоя карты: та же формула, что у coverage (известные клетки в маске / клеток маски)."""
        d = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        if d.shape == self.mask.shape:
            self.cov_layers[name] = int(((d >= 0) & self.mask).sum()) / max(self.n_mask, 1)
            self.layer_data[name] = d

    def _on_air_bid(self, msg):
        if msg.robot_id in self.air:
            self.air_done[msg.robot_id] = (self._now(), msg.done)

    def _log_pose(self, r, source, msg):
        """Строка в pose_log.csv, не чаще 5 Гц сим. времени на источник."""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t - self.log_last.get((r, source), -1e9) < 0.2:
            return
        self.log_last[(r, source)] = t
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose_log.write(f'{t:.3f},{r},{source},{p.x:.4f},{p.y:.4f},{yaw:.4f}\n')

    def _on_gt(self, r, msg):
        p = msg.pose.pose.position
        tr = self.gt_trails[r]
        if not tr or math.hypot(p.x - tr[-1][0], p.y - tr[-1][1]) > 0.1:
            tr.append((p.x, p.y))
        self._log_pose(r, 'gt', msg)

    def _on_odom(self, r, msg):
        self._log_pose(r, 'pose', msg)
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
                        + [f'{self.dist[r]:.2f}' for r in self.robots]
                        + ([f'{self.cov_layers["ground"]:.5f}', f'{self.cov_layers["air"]:.5f}'] if self.air else [])
                        + ([f'{self.cov_coll:.5f}'] if self.comms else []))
        self.f.flush()
        self.pose_log.flush()
        # все explorer сообщили done и держат это
        all_done = all(r in self.bids_done and t - self.bids_done[r][0] < 5.0 and self.bids_done[r][1]
                       for r in self.robots)
        # дроны: их explorer тоже должны сообщить done (топик /exploration/air_bids)
        all_done = all_done and all(r in self.air_done and t - self.air_done[r][0] < 5.0 and self.air_done[r][1]
                                    for r in self.air)
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
        held = self.done_since is not None and t - self.done_since >= float(self.get_parameter('done_hold').value)
        # причина завершения без учёта связи (как в Фазе 5)
        why = None
        if self.cov >= self.target and search_ok and (held or not self.air):
            why = f'покрытие {self.cov:.1%} >= {self.target:.0%}'
        elif self.done_since is not None and search_ok and held:
            why = 'все explorer сообщили done'
        # режим связи: перед завершением ждём, пока база получит всё, что роботы знали в этот момент
        # (версии слоёв карты и пачки наблюдений каждого источника), не дольше sync_timeout
        if why and self.comms:
            if self.sync_start is None:
                self.sync_start = t
                self.sync_target = self._sync_target()
                self.get_logger().info(f'SYNC: можно завершать ({why}), ждём, пока база получит данные роботов: {self.sync_target}')
            bm, bo = self.have.get('base', ({}, {}))
            if version_sync_ok(self.sync_target, bm, bo):
                sync_why = 'synced'
            elif t - self.sync_start >= float(self.get_parameter('sync_timeout').value):
                sync_why = 'timeout'
            else:
                sync_why = None     # ждём дальше (max_time ниже всё равно действует)
            if sync_why:
                self.get_logger().info(
                    f'SYNC: конец ожидания ({sync_why}), ждали {t - self.sync_start:.0f} с, '
                    f'база {100 * self.cov:.1f} %, коллективно {100 * self.cov_coll:.1f} %')
                self._finish(ts, why)
        elif why:
            self._finish(ts, why)
        elif self.comms:
            self.sync_start = None
        if not self.finished and self.max_time > 0 and ts >= self.max_time:
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
            with open(os.path.join(self.out_dir, 'final_map.yaml'), 'w') as f:
                yaml.safe_dump({'image': 'final_map.npy', 'resolution': float(self.res),
                                'origin': [float(self.ox), float(self.oy), 0.0],
                                'shape_hw': [int(self.data.shape[0]), int(self.data.shape[1])],
                                'row0': 'bottom'}, f)
        if self.comms and self.data_coll is not None:
            np.save(os.path.join(self.out_dir, 'final_map_collective.npy'), self.data_coll)
        # слои воздух/земля в той же сетке (origin и resolution как у final_map.yaml)
        for name, d in self.layer_data.items():
            np.save(os.path.join(self.out_dir, f'final_map_{name}.npy'), d)
        self.pose_log.flush()
        for name, trails in (('trajectories.csv', self.trails), ('gt_trajectories.csv', self.gt_trails)):
            with open(os.path.join(self.out_dir, name), 'w') as f:
                f.write('robot,x,y\n')
                for r, tr in trails.items():
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

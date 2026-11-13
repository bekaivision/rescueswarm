"""Узел explorer (по одному на робота): фронтиры на общей карте + децентрализованный аукцион.

Цикл раз в plan_period секунд (время симуляции):
 1. фронтиры на /map, цена каждого = длина пути - выигрыш (размер) + штраф за близость к целям других;
 2. публикация лучших K ставок на /exploration/bids;
 3. по таблице ставок всех роботов то же детерминированное жадное распределение (auction.assign);
 4. своя цель отправляется в Nav2 (NavigateToPose); при неудаче цель в чёрный список.

Роботы друг с другом: позы остальных приходят на /fleet/poses (rs_mapping/other_robots).
Если робот стоит на месте с целью, а рядом (block_dist) другой, срабатывает правило приоритета:
уступает робот с БОЛЬШИМ номером (отменяет цель, отъезжает от соседа, ждёт yield_hold с), робот с
меньшим номером просто ждёт. Неудача цели рядом с другим роботом (near_robot_dist) не заносит цель
в чёрный список надолго: помеха была в роботе, а не в цели (только на temp_blacklist_ttl с).
Все команды скорости (в т. ч. манёвры отсюда) идут в cmd_vel_nav -> collision_monitor -> cmd_vel.

Осмотр кандидатов (Фаза 3): /people/inspect_requests -> каждый открытый кандидат-человек становится
ставкой в том же аукционе с приоритетом над фронтирами (цена минус inspect_bonus, не больше одного
робота на кандидата). Цель = свободная клетка в 1.0-2.0 м от кандидата с прямой видимостью на карте,
курс на кандидата (rs_exploration/inspection.py). На месте: довернуть и постоять inspect_hold с, пока
people_fusion не решит судьбу кандидата. Неудача -> счётчик; после inspect_max_fails кандидат оставлен
(строка в логе, id уходит в gave_up ставок: так решают все роботы одинаково).
"""
import math
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseArray, PoseStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rs_interfaces.msg import ExplorationBid, PersonMarkArray
from visualization_msgs.msg import Marker, MarkerArray

from rs_exploration import auction, frontier, inspection

COLORS = {1: (1.0, 0.2, 0.2), 2: (0.2, 1.0, 0.3), 3: (0.3, 0.5, 1.0), 4: (1.0, 0.8, 0.1), 5: (0.9, 0.3, 1.0)}


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        P = self.declare_parameter
        P('robot_id', 'ground_1')
        P('region', [-11.0, -9.0, 11.0, 9.0])   # область поиска (xmin, ymin, xmax, ymax)
        P('plan_period', 1.0)
        P('num_bids', 8)
        P('assign_radius', 2.5)
        P('hysteresis', 3.0)          # бонус цене текущей цели, м
        P('bid_ttl', 5.0)             # ставка старше этого -> робот считается выбывшим
        P('min_cluster_cells', 8)
        P('min_unknown_cells', 40)
        P('clear_target', 0.3)
        P('gain_weight', 1.0)
        P('gain_cap', 3.0)            # м фронтира
        P('other_goal_radius', 3.0)
        P('other_goal_penalty', 6.0)
        P('blacklist_radius', 1.0)
        P('fail_blacklist_radius', 1.5)   # радиус для неудавшихся целей: соседние цели того же тупика тоже недостижимы
        P('blacklist_ttl', 600.0)
        P('goal_timeout', 150.0)
        P('stuck_time', 30.0)
        P('stuck_dist', 0.3)
        P('publish_cells', False)
        P('done_cycles', 3)
        P('start_stagger', 12.0)      # робот i стартует через (i-1)*stagger с после активации Nav2: вход узкий
        P('robots', ['ground_1', 'ground_2', 'ground_3'])               # все роботы (порядок = порядок поз в /fleet/poses)
        P('near_robot_dist', 2.0)     # неудача цели ближе этого к другому роботу = временная
        P('temp_blacklist_ttl', 20.0)
        P('block_time', 4.0)          # стоим с целью дольше этого рядом с роботом = заблокированы
        P('block_dist', 1.6)
        P('yield_hold', 8.0)          # после уступки новых целей не берём столько секунд
        P('yield_speed', 0.15)
        P('yield_time', 2.0)
        P('inspect_bonus', 1000.0)    # приоритет осмотра над фронтирами (вычитается из цены)
        P('inspect_r_min', 1.0)
        P('inspect_r_max', 2.0)
        P('inspect_r_pref', 1.5)
        P('inspect_hold', 4.0)        # стоим на месте после прибытия, с
        P('inspect_align_timeout', 6.0)
        P('inspect_max_fails', 2)     # столько неудач осмотра -> кандидат оставлен
        P('inspect_cooldown', 15.0)   # после неудачи этот робот столько секунд не берёт этого кандидата
        P('inspect_bearing_pen', 4.0)  # штраф точке с той же стороны, откуда уже смотрели, м
        P('inspect_max_bids', 4)
        P('inspect_noreach_s', 60.0)   # кандидат без достижимой точки осмотра дольше этого -> оставлен (даже при фронтирах)
        P('inspect_max_open_s', 300.0)  # кандидат открыт дольше этого (с первой заявки), не осматривается сейчас -> оставлен
        P('auction_delay', 0.3)       # аукцион через столько после публикации ставок: у всех свежая таблица
        # режим связи (Фаза 6): у каждого робота свой people_fusion, id меток локальные
        P('global_keys', False)       # ключи осмотра = id@x,y: одна и та же метка у разных роботов узнаётся по месту
        P('key_match_m', 1.2)         # ключи/кандидаты ближе этого считаются одним человеком
        # возврат на базу, когда всё исследовано
        P('home_on_done', False)
        P('home', [], ParameterDescriptor(dynamic_typing=True))   # [x, y, yaw] базы
        P('home_tol', 0.8)
        P('home_retries', 3)

        self.me = self.get_parameter('robot_id').value
        self.idx = int(self.me.split('_')[-1])
        self.region = list(self.get_parameter('region').value)
        self.K = int(self.get_parameter('num_bids').value)
        self.R = float(self.get_parameter('assign_radius').value)
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        self.map_msg = None
        self.pose = None
        self.yaw = 0.0
        self.fleet = {}     # robot_id -> (x, y) других роботов
        self.last_move_t = 0.0
        self.last_move_pose = None
        self.hold_until = 0.0
        self.block_logged = False
        self.table = {}      # robot_id -> (получено, сообщение)
        self.seq = 0
        self.round = 0
        self.goal = None     # (x, y) текущая цель
        self.goal_handle = None
        self.stale_handle = None
        self.goal_t0 = 0.0
        self.goal_id = 0
        self.progress = None  # (t, x, y) последняя точка заметного движения
        self.blacklist = []  # (x, y, до_какого_времени, радиус)
        self.empty_cycles = 0
        self.done = False
        # осмотр кандидатов
        self.requests = {}       # id -> (x, y) открытые кандидаты от people_fusion
        self.insp_fail = {}      # id -> число неудач этого робота
        self.insp_block = {}     # id -> до какого времени этот робот не берёт кандидата
        self.insp_bearings = {}  # id -> стороны, откуда уже смотрели
        self.gave_local = set()
        self.gave_pos = {}       # id -> (x, y) кандидата на момент отказа (для глобальных ключей)
        self.global_keys = bool(self.get_parameter('global_keys').value)
        self.key_match = float(self.get_parameter('key_match_m').value)
        # возврат на базу: отдельное состояние, не цель аукциона
        self.homing = False
        self.home_gid = 0
        self.home_handle = None
        self.home_fail = 0
        self.home_reached = False
        self.home_gave_up = False
        self.home_retry_at = 0.0
        self.home_t0 = 0.0
        self.home_progress = None
        self.giveup = inspection.GiveUp(float(self.get_parameter('inspect_noreach_s').value),
                                        float(self.get_parameter('inspect_max_open_s').value))
        self.req_ignore = {}     # id -> заявки со штампом не позже этого времени устарели (осмотр закончен)
        self.gave_all = set()
        self.open_ids = []
        self.goal_key = ''       # id кандидата, если текущая цель = осмотр
        self.insp = None         # None | dict(state, t0, t_hold)
        self.insp_att = 0
        self.trail = []
        self.prof = {}
        self.stats = dict(yielded=0, waited=0, temp_fail=0, unstick=0, sent=0, reached=0, failed=0, stuck=0, timeout=0, switched=0)

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(Odometry, 'pose', self._on_odom, 10)
        self.create_subscription(PoseArray, '/fleet/poses', self._on_fleet, 5)
        self.create_subscription(ExplorationBid, '/exploration/bids', self._on_bid, 20)
        self.create_subscription(PersonMarkArray, '/people/inspect_requests', self._on_requests, 5)
        self.bid_pub = self.create_publisher(ExplorationBid, '/exploration/bids', 20)
        self.mark_pub = self.create_publisher(MarkerArray, 'explorer/frontiers', 1)
        self.goal_pub = self.create_publisher(PoseStamped, 'explorer/goal', 1)
        self.trail_pub = self.create_publisher(Path, 'explorer/trail', 1)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # цели шлём только когда Nav2 (bt_navigator) активен, иначе он их сразу отвергает
        self.nav_active = False
        self.state_cli = self.create_client(GetState, 'bt_navigator/get_state')
        self.state_busy = False
        self.lc_cli = self.create_client(ManageLifecycleNodes, 'lifecycle_manager_navigation/manage_nodes')
        self.nav_wait_t0 = None
        self.nav_active_t = 0.0
        self.nav_retry_at = 45.0   # сек. ожидания до первой повторной попытки запуска Nav2
        # снятие с мели: после серии неудач без движения - назад и поворот по cmd_vel
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)   # через collision_monitor
        self.fail_streak = 0
        self.streak_pose = None
        self.streak_entries = []
        self.unsticking = False
        self.create_timer(float(self.get_parameter('plan_period').value), self._cycle)
        self.create_timer(1.0, self._publish_trail)
        self.create_timer(0.5, self._yield_tick)
        self.create_timer(0.1, self._inspect_tick)
        self.create_timer(30.0, self._log_prof)
        self.get_logger().info(f'explorer {self.me} запущен, область {self.region}')

    # --- вспомогательное ----------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _tic(self, key, t0):
        self.prof[key] = self.prof.get(key, 0.0) + time.process_time() - t0

    def _log_prof(self):
        self.get_logger().info('CPU за 30 с, с: ' + ', '.join(f'{k}={v:.2f}' for k, v in self.prof.items()))
        self.prof = {}

    def _on_map(self, msg):
        t0 = time.process_time()
        self.map_msg = msg
        self._tic('map_cb', t0)

    def _on_odom(self, msg):
        t0 = time.process_time()
        self._on_odom_impl(msg)
        self._tic('odom', t0)

    def _on_odom_impl(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.last_move_pose is None or math.hypot(p.x - self.last_move_pose[0], p.y - self.last_move_pose[1]) > 0.15:
            self.last_move_pose = (p.x, p.y)
            self.last_move_t = self._now()
        if not self.trail or math.hypot(p.x - self.trail[-1][0], p.y - self.trail[-1][1]) > 0.2:
            self.trail.append((p.x, p.y))

    def _on_fleet(self, msg):
        ids = list(self.get_parameter('robots').value)
        self.fleet = {rid: (p.position.x, p.position.y) for rid, p in zip(ids, msg.poses)
                      if rid != self.me and math.isfinite(p.position.x)}

    def _nearest_other(self):
        """(расстояние, id, (x, y)) ближайшего другого робота или (inf, None, None)."""
        best = (float('inf'), None, None)
        if self.pose is None:
            return best
        for rid, (x, y) in self.fleet.items():
            d = math.hypot(x - self.pose[0], y - self.pose[1])
            if d < best[0]:
                best = (d, rid, (x, y))
        return best

    def _on_bid(self, msg):
        if msg.robot_id != self.me:
            self.table[msg.robot_id] = (self._now(), msg)

    def _blacklisted(self, x, y, now):
        return any(now < t and math.hypot(x - bx, y - by) < r for bx, by, t, r in self.blacklist)

    def _add_blacklist(self, x, y, why, ttl=None, track=True):
        ttl = float(self.get_parameter('blacklist_ttl').value) if ttl is None else ttl
        rad = float(self.get_parameter('fail_blacklist_radius' if track else 'blacklist_radius').value)
        self.blacklist.append((x, y, self._now() + ttl, rad))
        if track:
            self.streak_entries.append(self.blacklist[-1])
        self.get_logger().warn(f'чёрный список ({why}): ({x:.1f}, {y:.1f}) на {ttl:.0f} с, всего {len(self.blacklist)}')

    # --- основной цикл ------------------------------------------------------
    def _cycle(self):
        if self.map_msg is None or self.pose is None:
            return
        t0 = time.process_time()
        now = self._now()
        m = self.map_msg
        h, w = m.info.height, m.info.width
        data = np.asarray(m.data, dtype=np.int8).reshape(h, w)
        self._tic('map_np', t0)
        t0 = time.process_time()
        if (data == 0).sum() < 50:
            return  # карты ещё нет
        gp = self.get_parameter
        an = frontier.analyse(
            data, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y, self.region,
            min_unknown=int(gp('min_unknown_cells').value), min_cluster=int(gp('min_cluster_cells').value),
            clear_target=float(gp('clear_target').value))
        self._tic('analyse', t0)
        rx, ry = self.pose

        # актуальные ставки других (старые = робот выбыл)
        ttl = float(gp('bid_ttl').value)
        for rid in [k for k, (t, _) in self.table.items() if now - t > ttl]:
            del self.table[rid]
            self.get_logger().warn(f'ставки {rid} устарели: считаем выбывшим')

        # проверка текущей цели
        self._check_goal(an, now)
        self.gave_all = set(self.gave_local)
        if self.global_keys:
            # чужие отказы приходят как id@x,y: сравниваем по месту, а не по чужому id
            foreign = [kp for _, msg in self.table.values() for kp in map(auction.key_pos, msg.gave_up) if kp]
            for rid, (qx, qy) in self.requests.items():
                if any(math.hypot(qx - fx, qy - fy) < self.key_match for fx, fy in foreign):
                    self.gave_all.add(rid)
        else:
            for _, msg in self.table.values():
                self.gave_all.update(msg.gave_up)
        self.open_ids = [k for k in self.requests if k not in self.gave_all]

        # кандидаты
        cands = []   # (cost, x, y, size)
        if len(an.tx):
            keep = np.array([not self._blacklisted(x, y, now) for x, y in zip(an.tx, an.ty)])
            tx, ty, sz = an.tx[keep], an.ty[keep], an.size[keep]
            if len(tx):
                pl = frontier.path_costs(an, rx, ry, tx, ty)
                if not np.isfinite(pl).any():
                    pl = np.hypot(tx - rx, ty - ry) * 2.0   # запасной вариант: прямая x2
                others = [(msg.current_goal.x, msg.current_goal.y)
                          for _, msg in self.table.values() if msg.has_goal]
                gain = np.minimum(sz * an.res, float(gp('gain_cap').value))
                cost = pl - float(gp('gain_weight').value) * gain
                orad = float(gp('other_goal_radius').value)
                for i in range(len(tx)):
                    if not np.isfinite(pl[i]):
                        continue
                    c = cost[i]
                    if any(math.hypot(tx[i] - ox, ty[i] - oy) < orad for ox, oy in others):
                        c += float(gp('other_goal_penalty').value)
                    x, y = float(tx[i]), float(ty[i])
                    if self.goal is not None and math.hypot(x - self.goal[0], y - self.goal[1]) < 1.5:
                        c -= float(gp('hysteresis').value)
                        x, y = self.goal
                    cands.append((float(np.float32(c)), x, y, float(sz[i])))
        cands.sort()
        top = [(c, x, y, sz, '') for c, x, y, sz in cands[:self.K]]
        insp_bids = self._inspection_bids(an, data, now, no_frontiers=not cands)
        top = insp_bids + top

        # состояние "готово"
        if not cands and self.goal is None and not self.open_ids:
            self.empty_cycles += 1
            if self.empty_cycles >= int(gp('done_cycles').value) and not self.done:
                self.done = True
                self.get_logger().info(f'exploration done: фронтиров нет, цели: {self.stats}')
        else:
            self.empty_cycles = 0
            if self.done:
                self.done = False
                self.get_logger().info('фронтиры появились снова: продолжаем')
                self._cancel_home()
                self.home_reached = False
                self.home_gave_up = False
                self.home_fail = 0

        self._home_step(now)
        self._publish_bid(top)
        self._publish_markers(an, cands)

        # аукцион: чуть позже, когда ставки остальных этого раунда уже дошли
        self.top, self.ncands, self.cycle_now = top, len(cands), now
        holder = []

        def run_auction():
            holder[0].cancel()
            self.destroy_timer(holder[0])
            self._auction()
        holder.append(self.create_timer(float(gp('auction_delay').value), run_auction))

    def _auction(self):
        top, now = self.top, self._now()
        table = {self.me: [(c, x, y, self._gkey(k)) for c, x, y, _, k in top]}
        for rid, (_, msg) in self.table.items():
            ids = list(msg.target_ids) + [''] * len(msg.targets)
            table[rid] = [(float(c), p.x, p.y, k) for c, p, k in zip(msg.costs, msg.targets, ids)]
        self.my_keys = {(x, y): k for _, x, y, _, k in top}   # локальные id: по позиции цели находим свой id
        res = auction.assign(table, self.R, self.key_match if self.global_keys else 0.0)
        self.round += 1
        desc = ' '.join(f'{rid[-1]}->({x:.1f},{y:.1f})c{c:.1f}' for rid, (x, y, c) in sorted(res.items()))
        mine = res.get(self.me)
        self._act(mine, now)
        self._flush_cancel()
        sz_t = sum(len(v) for v in table.values())
        goal_s = f'({self.goal[0]:.1f},{self.goal[1]:.1f})' if self.goal else '-'
        self.get_logger().info(f'аукцион #{self.round}: робот={len(table)} ставок={sz_t} фронтиров={self.ncands} | {desc or "нет"} | моя цель {goal_s}')

    def _check_goal(self, an, now):
        if self.goal is None:
            return
        gx, gy = self.goal
        if self.goal_key:
            self._check_inspect_goal(now)
            return
        # цель больше не фронтир (уже исследована)?
        near = False
        if an.frontier.any():
            fy, fx = np.nonzero(an.frontier)
            wx, wy = an.world(fx, fy)
            near = bool(np.min(np.hypot(wx - gx, wy - gy)) < 1.5)
        if not near:
            self.get_logger().info(f'цель ({gx:.1f}, {gy:.1f}) уже не фронтир: отмена')
            self._cancel_goal()
            return
        if now - self.goal_t0 > float(self.get_parameter('goal_timeout').value):
            self.stats['timeout'] += 1
            self._cancel_goal()
            self._fail_goal(gx, gy, 'тайм-аут цели')
            return
        t, px, py = self.progress
        if math.hypot(self.pose[0] - px, self.pose[1] - py) > float(self.get_parameter('stuck_dist').value):
            self.progress = (now, self.pose[0], self.pose[1])
        elif now - t > float(self.get_parameter('stuck_time').value):
            self.stats['stuck'] += 1
            self._cancel_goal()
            self._fail_goal(gx, gy, 'застрял')

    def _act(self, mine, now):
        if mine is None:
            return   # нет назначения: продолжаем текущую цель, иначе ждём
        x, y, _ = mine
        key = getattr(self, 'my_keys', {}).get((x, y), '')
        if self.goal is not None and key == self.goal_key and math.hypot(x - self.goal[0], y - self.goal[1]) < 1.0:
            return   # то же самое
        if self.unsticking or now < self.hold_until:
            return
        if not self.nav_active:
            self._poll_nav_state()
            return
        if not self.nav.server_is_ready():
            return
        if now - self.nav_active_t < (self.idx - 1) * float(self.get_parameter('start_stagger').value):
            return
        if self.goal is not None:
            self.stats['switched'] += 1
        self._send_goal(x, y, key)

    # --- роботы друг с другом -------------------------------------------------
    def _fail_goal(self, x, y, why):
        """Неудача цели. Рядом другой робот: виноват он, цель блокируем ненадолго и без снятия с мели."""
        d, rid, _ = self._nearest_other()
        if self.goal_key:
            key = self.goal_key
            if d < float(self.get_parameter('near_robot_dist').value):
                self.stats['temp_fail'] += 1
                self.insp_block[key] = self._now() + float(self.get_parameter('temp_blacklist_ttl').value)
                self.get_logger().warn(f'осмотр {key}: {why}, рядом {rid} в {d:.1f} м: временно')
            else:
                self._inspect_fail(key, why)
                self._note_failure()
            return
        if d < float(self.get_parameter('near_robot_dist').value):
            self.stats['temp_fail'] += 1
            self._add_blacklist(x, y, f'{why}, рядом {rid} в {d:.1f} м: временно',
                                ttl=float(self.get_parameter('temp_blacklist_ttl').value), track=False)
            return
        self._add_blacklist(x, y, why)
        self._note_failure()

    def _yield_tick(self):
        """Правило приоритета: стоим с целью рядом с другим роботом - уступает больший номер."""
        if self.pose is None or self.goal is None or self.unsticking:
            self.block_logged = False
            return
        now = self._now()
        gp = self.get_parameter
        if now - self.last_move_t < float(gp('block_time').value):
            self.block_logged = False
            return
        d, rid, pos = self._nearest_other()
        if d > float(gp('block_dist').value):
            return
        if int(rid.split('_')[-1]) < self.idx:
            # приоритет у соседа: уступаем
            self.stats['yielded'] += 1
            gx, gy = self.goal
            self.get_logger().warn(f'заблокирован роботом {rid} ({d:.1f} м): уступаю, цель ({gx:.1f}, {gy:.1f}) на паузу')
            if self.goal_key:
                self.insp_block[self.goal_key] = now + float(gp('temp_blacklist_ttl').value)
            else:
                self.blacklist.append((gx, gy, now + float(gp('temp_blacklist_ttl').value), float(gp('blacklist_radius').value)))
            self._cancel_goal()
            self._flush_cancel()
            self.hold_until = now + float(gp('yield_hold').value)
            ahead = (pos[0] - self.pose[0]) * math.cos(self.yaw) + (pos[1] - self.pose[1]) * math.sin(self.yaw)
            v = float(gp('yield_speed').value)
            self._start_motion(-v if ahead > 0 else v, 0.0, float(gp('yield_time').value), 'отъезд от робота')
        elif not self.block_logged:
            # приоритет у меня: стою и жду, пока сосед уступит
            self.block_logged = True
            self.stats['waited'] += 1
            self.get_logger().info(f'жду: робот {rid} ({d:.1f} м) мешает, у меня приоритет')

    def _start_motion(self, vx, wz, dur, label):
        """Короткий манёвр по cmd_vel_nav (через collision_monitor)."""
        self.unsticking = True
        t0 = self._now()
        holder = []

        def tick():
            msg = Twist()
            if self._now() - t0 < dur:
                msg.linear.x, msg.angular.z = vx, wz
                self.cmd_pub.publish(msg)
                return
            self.cmd_pub.publish(msg)
            holder[0].cancel()
            self.destroy_timer(holder[0])
            self.unsticking = False
            self.get_logger().info(f'{label}: закончено')
        holder.append(self.create_timer(0.1, tick))

    # --- осмотр кандидатов -----------------------------------------------------
    def _on_requests(self, msg):
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.requests = {m.id: (m.position.x, m.position.y) for m in msg.marks
                         if st > self.req_ignore.get(m.id, -1.0)}

    def _inspection_bids(self, an, data, now, no_frontiers):
        """Ставки на осмотр открытых кандидатов: [(цена - бонус, x, y, 0, id)], лучшая точка на кандидата."""
        gp = self.get_parameter
        rx, ry = self.pose
        pts = []   # (id, x, y, bearing, штраф)
        by_id = {}
        for key in self.open_ids:
            if now < self.insp_block.get(key, 0.0):
                continue
            cx, cy = self.requests[key]
            sp = inspection.standing_points(
                an, data, cx, cy, r_min=float(gp('inspect_r_min').value), r_max=float(gp('inspect_r_max').value),
                r_pref=float(gp('inspect_r_pref').value))
            if self.goal_key == key and self.goal is not None:
                gx, gy = self.goal
                sp.append((gx, gy, math.atan2(cy - gy, cx - gx), math.atan2(gy - cy, gx - cx), -float(gp('hysteresis').value)))
            used = self.insp_bearings.get(key, [])
            pen = float(gp('inspect_bearing_pen').value)
            for s in sp:
                extra = s[4] if len(s) > 4 else 0.0
                if any(inspection.angle_diff(s[3], b) < 0.9 for b in used):
                    extra += pen
                pts.append((key, s[0], s[1], extra))
            by_id[key] = len(sp)
        out = []
        if pts:
            pl = frontier.path_costs(an, rx, ry, [p[1] for p in pts], [p[2] for p in pts])
            best = {}
            for (key, x, y, extra), l in zip(pts, pl):
                if not np.isfinite(l):
                    continue
                c = l + extra
                if key not in best or c < best[key][0]:
                    best[key] = (c, x, y)
            bonus = float(gp('inspect_bonus').value)
            out = [(float(np.float32(c - bonus)), x, y, 0.0, key) for key, (c, x, y) in best.items()]
            out.sort()
            out = out[:int(gp('inspect_max_bids').value)]
            reachable = set(best)
        else:
            reachable = set()
        # нет точки осмотра, а исследовать больше нечего: считаем неудачей (иначе отказались бы зря рано)
        if no_frontiers:
            for key in self.open_ids:
                if key not in reachable and now >= self.insp_block.get(key, 0.0) and key != self.goal_key:
                    self._inspect_fail(key, 'нет достижимой точки осмотра', cooldown=5.0)
        # фронтиры ещё есть, но к кандидату никто не может подъехать (человек за дверью, за стеной):
        # ограниченное ожидание, иначе завершение ждёт его вечно
        blocked = {k for k in self.open_ids if now < self.insp_block.get(k, 0.0)}
        for key, why in self.giveup.update(now, self.open_ids, reachable, self.goal_key, blocked):
            self._inspect_giveup(key, why)
        self.open_ids = [k for k in self.open_ids if k not in self.gave_local]
        return out

    def _mark_gave(self, key):
        """Кандидат оставлен: запоминаем и позицию (для глобального ключа в ставках)."""
        self.gave_local.add(key)
        if key in self.requests:
            self.gave_pos[key] = self.requests[key]

    def _gkey(self, key):
        """Ключ для ставок: в глобальном режиме id@x,y по позиции кандидата, иначе локальный id."""
        if not key or not self.global_keys or key not in self.requests:
            return key
        return auction.make_key(key, *self.requests[key])

    def _inspect_giveup(self, key, why):
        if key not in self.gave_local:
            self._mark_gave(key)
            self.get_logger().warn(f'кандидат {key} оставлен: {why}')

    def _inspect_fail(self, key, why, cooldown=None):
        gp = self.get_parameter
        now = self._now()
        n = self.insp_fail.get(key, 0) + 1
        self.insp_fail[key] = n
        self.insp_block[key] = now + (float(gp('inspect_cooldown').value) if cooldown is None else cooldown)
        self.get_logger().warn(f'осмотр {key} не удался ({why}), неудач {n}/{int(gp("inspect_max_fails").value)}')
        if n >= int(gp('inspect_max_fails').value) and key not in self.gave_local:
            self._mark_gave(key)
            self.get_logger().warn(f'кандидат {key} оставлен после {n} неудач осмотра')

    def _check_inspect_goal(self, now):
        key = self.goal_key
        gx, gy = self.goal
        gp = self.get_parameter
        if key not in self.requests or key in self.gave_all:
            self.get_logger().info(f'осмотр {key}: кандидат решён или оставлен, отмена цели')
            self._cancel_goal()
            return
        if self.insp is not None:
            return   # на месте: ведёт _inspect_tick
        if now - self.goal_t0 > float(gp('goal_timeout').value):
            self.stats['timeout'] += 1
            self._cancel_goal()
            self.goal_key = key
            self._fail_goal(gx, gy, 'тайм-аут цели')
            self.goal_key = ''
            return
        t, px, py = self.progress
        if math.hypot(self.pose[0] - px, self.pose[1] - py) > float(gp('stuck_dist').value):
            self.progress = (now, self.pose[0], self.pose[1])
        elif now - t > float(gp('stuck_time').value):
            self.stats['stuck'] += 1
            self._cancel_goal()
            self.goal_key = key
            self._fail_goal(gx, gy, 'застрял')
            self.goal_key = ''

    def _start_inspect_hold(self):
        now = self._now()
        key = self.goal_key
        self.insp = dict(state='align', t0=now, t_hold=None)
        self.last_move_t = now
        cx, cy = self.requests.get(key, self.goal)
        self.insp_bearings.setdefault(key, []).append(math.atan2(self.pose[1] - cy, self.pose[0] - cx))
        self.get_logger().info(f'осмотр {key}: на месте, доворот и выдержка')

    def _inspect_tick(self):
        """На месте осмотра: довернуть на кандидата, постоять; если кандидат решён - сразу дальше."""
        if self.insp is None:
            return
        gp = self.get_parameter
        now = self._now()
        key = self.goal_key
        if self.goal is None or not key:
            self.insp = None
            return
        if key not in self.requests or key in self.gave_all:
            self.get_logger().info(f'осмотр {key}: кандидат решён во время осмотра')
            self.insp_fail.pop(key, None)
            self._finish_inspect()
            return
        cx, cy = self.requests[key]
        err = (math.atan2(cy - self.pose[1], cx - self.pose[0]) - self.yaw + math.pi) % (2 * math.pi) - math.pi
        self.last_move_t = now   # стоим намеренно: правило "заблокирован" не срабатывает
        st = self.insp
        if st['state'] == 'align':
            if abs(err) < 0.12 or now - st['t0'] > float(gp('inspect_align_timeout').value):
                st['state'], st['t_hold'] = 'hold', now
                self.cmd_pub.publish(Twist())
            else:
                msg = Twist()
                msg.angular.z = float(np.clip(1.5 * err, -0.6, 0.6))
                self.cmd_pub.publish(msg)
        elif now - st['t_hold'] > float(gp('inspect_hold').value):
            self._inspect_fail(key, 'после выдержки кандидат не решён')
            self._finish_inspect()

    def _finish_inspect(self):
        key = self.goal_key
        if key:
            # заявка могла устареть в пути: снимаем и не принимаем более старые сообщения
            self.requests.pop(key, None)
            self.req_ignore[key] = self._now()
        self.insp = None
        self.cmd_pub.publish(Twist())
        self.goal = None
        self.goal_key = ''
        self.goal_handle = None
        self.goal_id += 1

    # --- Nav2 ---------------------------------------------------------------
    def _reset_streak(self):
        self.fail_streak = 0
        self.streak_pose = None
        self.streak_entries = []

    def _note_failure(self):
        if self.streak_pose is None or math.hypot(self.pose[0] - self.streak_pose[0], self.pose[1] - self.streak_pose[1]) > 1.0:
            self.fail_streak = 0
            self.streak_pose = self.pose
            self.streak_entries = self.streak_entries[-1:]
        self.fail_streak += 1
        if self.fail_streak >= 2 and not self.unsticking:
            self._unstick()

    def _unstick(self):
        # робот не двигается, а цели падают: виноваты не цели. Возвращаем их из чёрного списка.
        self.unsticking = True
        self.stats['unstick'] += 1
        for e in self.streak_entries:
            if e in self.blacklist:
                self.blacklist.remove(e)
        self.get_logger().warn(f'застряли ({self.fail_streak} неудач подряд без движения): сдаём назад и поворачиваем')
        self._reset_streak()
        # Nav2 BackUp/Spin не помогают, если корпус уже касается препятствия (footprint в
        # лидарной зоне): их проверка столкновений отказывает. Едем вслепую по cmd_vel.
        self.unstick_t0 = self._now()
        self.unstick_dir = -self.unstick_dir if hasattr(self, 'unstick_dir') else 1.0
        self.unstick_timer = self.create_timer(0.1, self._unstick_tick)

    def _unstick_tick(self):
        t = self._now() - self.unstick_t0
        msg = Twist()
        if t < 2.5:
            msg.linear.x = -0.15
        elif t < 4.5:
            msg.angular.z = 1.0 * self.unstick_dir
        else:
            self.cmd_pub.publish(msg)
            self.unstick_timer.cancel()
            self.destroy_timer(self.unstick_timer)
            self.unsticking = False
            self.get_logger().info('снятие с мели закончено')
            return
        self.cmd_pub.publish(msg)

    def _poll_nav_state(self):
        # редкий сбой старта: lifecycle-переход потерялся и узел навсегда 'unconfigured'
        if self.nav_wait_t0 is None:
            self.nav_wait_t0 = self._now()
        if self._now() - self.nav_wait_t0 > self.nav_retry_at and self.lc_cli.service_is_ready():
            self.nav_retry_at += 45.0
            # Сначала RESET, потом STARTUP: если первый запуск оборвался на середине, часть узлов уже
            # active, и голый STARTUP навсегда падает на "configure" активного controller_server (Фаза 5, seed 1).
            req = ManageLifecycleNodes.Request()
            req.command = ManageLifecycleNodes.Request.RESET

            def after_reset(_f):
                s = ManageLifecycleNodes.Request()
                s.command = ManageLifecycleNodes.Request.STARTUP
                self.lc_cli.call_async(s)
            self.lc_cli.call_async(req).add_done_callback(after_reset)
            self.get_logger().warn('Nav2 не стал активным: RESET и повторный запуск через lifecycle_manager')
        if self.state_busy or not self.state_cli.service_is_ready():
            return
        self.state_busy = True
        fut = self.state_cli.call_async(GetState.Request())

        def done(f):
            self.state_busy = False
            r = f.result()
            if r is not None and r.current_state.id == State.PRIMARY_STATE_ACTIVE:
                self.nav_active = True
                self.nav_active_t = self._now()
                self.get_logger().info('Nav2 активен, начинаем исследование')
        fut.add_done_callback(done)

    def _send_goal(self, x, y, key=''):
        if self.homing:
            # обычная цель вытесняет домашнюю в Nav2; результат домашней больше не нужен
            self.homing = False
            self.home_gid += 1
            self.home_handle = None
        self.goal_id += 1
        self.insp = None
        gid = self.goal_id
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'world'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = x
        g.pose.pose.position.y = y
        yaw = math.atan2(y - self.pose[1], x - self.pose[0])
        if key and key in self.requests:
            yaw = math.atan2(self.requests[key][1] - y, self.requests[key][0] - x)
        self.goal_key = key
        g.pose.pose.orientation.z = math.sin(yaw / 2)
        g.pose.pose.orientation.w = math.cos(yaw / 2)
        self.stale_handle = None   # новая цель вытеснит старую в Nav2
        self.goal = (x, y)
        self.goal_handle = None
        self.goal_t0 = self._now()
        self.progress = (self.goal_t0, self.pose[0], self.pose[1])
        self.last_move_t = self.goal_t0
        self.stats['sent'] += 1
        self.goal_pub.publish(g.pose)
        what = f' осмотр {key}' if key else ''
        self.get_logger().info(f'цель #{gid}: ({x:.1f}, {y:.1f}){what}')
        fut = self.nav.send_goal_async(g)
        fut.add_done_callback(lambda f, gid=gid: self._on_accept(f, gid))

    def _on_accept(self, fut, gid):
        gh = fut.result()
        if gid != self.goal_id:
            if gh.accepted:
                gh.cancel_goal_async()   # цель успели отменить до принятия
            return
        if not gh.accepted:
            self.stats['failed'] += 1
            if self.goal_key:
                self._inspect_fail(self.goal_key, 'Nav2 отклонил цель')
            else:
                self._add_blacklist(*self.goal, 'Nav2 отклонил цель')
            self.goal = None
            self.goal_key = ''
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f, gid=gid: self._on_result(f, gid))

    def _on_result(self, fut, gid):
        if gid != self.goal_id or self.goal is None:
            return   # устаревший или отменённый нами результат
        status = fut.result().status
        gx, gy = self.goal
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.stats['reached'] += 1
            self.get_logger().info(f'цель #{gid} достигнута')
            self._reset_streak()
            if self.goal_key:
                self._start_inspect_hold()
                return   # цель остаётся активной до конца осмотра
            # если фронтир на месте остался, он бесполезен: исключаем
            self.blacklist.append((gx, gy, self._now() + float(self.get_parameter('blacklist_ttl').value),
                                   float(self.get_parameter('blacklist_radius').value)))
        else:
            self.stats['failed'] += 1
            self._fail_goal(gx, gy, f'Nav2 статус {status}')
        self.goal = None
        self.goal_key = ''
        self.goal_handle = None

    def _cancel_goal(self):
        # отмену откладываем до конца аукциона: если сразу пойдёт новая цель, Nav2 сам
        # вытеснит старую (отмена + новая цель подряд = новая цель прерывается)
        if self.goal_handle is not None:
            self.stale_handle = self.goal_handle
        self.goal = None
        self.goal_key = ''
        self.insp = None
        self.goal_handle = None
        self.goal_id += 1   # результат отменённой цели игнорируем

    # --- возврат на базу -------------------------------------------------------
    def _home_pose(self):
        h = self.get_parameter('home').value
        if h is None or len(h) < 2:
            return None
        return float(h[0]), float(h[1]), float(h[2]) if len(h) > 2 else 0.0

    def _cancel_home(self):
        """Отмена домашней цели (отложенная, как у обычной: если сразу пойдёт новая, Nav2 вытеснит сам)."""
        if not self.homing:
            return
        if self.home_handle is not None:
            self.stale_handle = self.home_handle
        self.homing = False
        self.home_handle = None
        self.home_gid += 1

    def _home_fail(self, why):
        """Неудача возврата: без чёрного списка и без счётчиков целей фронтира."""
        self.home_fail += 1
        retries = int(self.get_parameter('home_retries').value)
        self.home_retry_at = self._now() + 10.0
        self.get_logger().warn(f'возврат на базу не удался ({why}), неудач {self.home_fail}/{retries}')
        if self.home_fail >= retries and not self.home_gave_up:
            self.home_gave_up = True
            self.get_logger().warn('возврат на базу: попытки кончились, остаёмся на месте')

    def _home_step(self, now):
        """Раз в цикл: следим за домашней целью или запускаем её (только когда робот done и без цели)."""
        gp = self.get_parameter
        home = self._home_pose()
        if not gp('home_on_done').value or home is None or self.pose is None:
            return
        dist = math.hypot(home[0] - self.pose[0], home[1] - self.pose[1])
        if self.homing:
            if not self.done:
                return   # отмену сделает ветка 'фронтиры появились снова'
            if dist <= float(gp('home_tol').value):
                self._cancel_home()
                self.home_reached = True
                self.get_logger().info('на базе')
                return
            t, px, py = self.home_progress
            if math.hypot(self.pose[0] - px, self.pose[1] - py) > float(gp('stuck_dist').value):
                self.home_progress = (now, self.pose[0], self.pose[1])
            elif now - t > float(gp('stuck_time').value):
                self._cancel_home()
                self._home_fail('застрял')
                self._note_failure()
            elif now - self.home_t0 > float(gp('goal_timeout').value):
                self._cancel_home()
                self._home_fail('тайм-аут')
            return
        if self.done and not self.home_reached and dist <= float(gp('home_tol').value):
            self.home_reached = True
            self.get_logger().info('на базе')
            return
        if not auction.need_home(self.done, self.goal is not None, self.homing, self.home_reached,
                                 self.home_fail, int(gp('home_retries').value), dist, float(gp('home_tol').value)):
            return
        if now < self.home_retry_at or self.unsticking or now < self.hold_until:
            return
        if not self.nav_active or not self.nav.server_is_ready():
            return
        self._send_home(home)

    def _send_home(self, home):
        x, y, yaw = home
        self.home_gid += 1
        gid = self.home_gid
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'world'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = x
        g.pose.pose.position.y = y
        g.pose.pose.orientation.z = math.sin(yaw / 2)
        g.pose.pose.orientation.w = math.cos(yaw / 2)
        self.homing = True
        self.home_handle = None
        self.home_t0 = self._now()
        self.home_progress = (self.home_t0, self.pose[0], self.pose[1])
        self.get_logger().info(f'возврат на базу: ({x:.1f}, {y:.1f})')
        fut = self.nav.send_goal_async(g)
        fut.add_done_callback(lambda f, gid=gid: self._on_home_accept(f, gid))

    def _on_home_accept(self, fut, gid):
        gh = fut.result()
        if gid != self.home_gid:
            if gh.accepted:
                gh.cancel_goal_async()
            return
        if not gh.accepted:
            self.homing = False
            self._home_fail('Nav2 отклонил цель')
            return
        self.home_handle = gh
        gh.get_result_async().add_done_callback(lambda f, gid=gid: self._on_home_result(f, gid))

    def _on_home_result(self, fut, gid):
        if gid != self.home_gid or not self.homing:
            return   # устаревший или отменённый нами результат
        self.homing = False
        self.home_handle = None
        if fut.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.home_reached = True
            self.get_logger().info('на базе')
        else:
            self._home_fail(f'Nav2 статус {fut.result().status}')

    def _flush_cancel(self):
        gh, self.stale_handle = self.stale_handle, None
        if gh is not None and self.goal is None:
            gh.cancel_goal_async()

    # --- публикации ---------------------------------------------------------
    def _publish_bid(self, top):
        msg = ExplorationBid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.robot_id = self.me
        self.seq += 1
        msg.seq = self.seq
        msg.has_goal = self.goal is not None
        if self.goal:
            msg.current_goal = Point(x=self.goal[0], y=self.goal[1])
        msg.targets = [Point(x=x, y=y) for _, x, y, _, _ in top]
        msg.costs = [c for c, *_ in top]
        msg.target_ids = [self._gkey(k) for *_, k in top]
        msg.open_candidates = len(self.open_ids)
        if self.global_keys:
            msg.gave_up = sorted(auction.make_key(k, *self.gave_pos[k]) for k in self.gave_local if k in self.gave_pos)
        else:
            msg.gave_up = sorted(self.gave_local)
        msg.done = self.done
        self.bid_pub.publish(msg)

    def _publish_markers(self, an, cands):
        col = COLORS.get(self.idx, (1.0, 1.0, 1.0))
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = 'world'
        m.ns = f'{self.me}_candidates'
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color.r, m.color.g, m.color.b, m.color.a = (*col, 0.9)
        m.pose.orientation.w = 1.0
        m.points = [Point(x=x, y=y, z=0.2) for _, x, y, _ in cands]
        arr.markers.append(m)
        if self.get_parameter('publish_cells').value and an.frontier.any():
            c = Marker()
            c.header.frame_id = 'world'
            c.ns = 'frontier_cells'
            c.id = 1
            c.type = Marker.POINTS
            c.action = Marker.ADD
            c.scale.x = c.scale.y = 0.06
            c.color.r = c.color.g = c.color.b = 1.0
            c.color.a = 0.7
            c.pose.orientation.w = 1.0
            fy, fx = np.nonzero(an.frontier)
            wx, wy = an.world(fx[::2], fy[::2])
            c.points = [Point(x=float(a), y=float(b), z=0.05) for a, b in zip(wx, wy)]
            arr.markers.append(c)
        self.mark_pub.publish(arr)

    def _publish_trail(self):
        if not self.trail:
            return
        t0 = time.process_time()
        self._publish_trail_impl()
        self._tic('trail', t0)

    def _publish_trail_impl(self):
        p = Path()
        p.header.frame_id = 'world'
        p.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.trail[-3000:]:
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self.trail_pub.publish(p)


def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

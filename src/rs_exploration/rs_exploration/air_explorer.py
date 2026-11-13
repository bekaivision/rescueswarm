"""Узел air_explorer (по одному на дрон): обзор области сверху по карте /map_air + аукцион между дронами.

Цикл раз в plan_period секунд (время симуляции):
 1. крупные клетки области, которые ещё не видны сверху (rs_exploration/air_targets.py);
 2. ставки лучших K клеток на /exploration/air_bids (наземные роботы этот топик не читают);
 3. то же детерминированное жадное распределение (auction.assign) по ставкам всех дронов;
 4. своя цель уходит в NavigateToPose дрона; цели шлём только в состояниях READY и MOVING.

Клетка облетена, когда действие завершилось SUCCEEDED (больше не берём). После ABORTED клетку
пробуем ещё max_retries раз, затем она проваленная. Когда кандидатов нет done_cycles циклов подряд,
дрон летит в точку старта (где впервые сообщил READY), вызывает land и дальше публикует ставки
с done = true (чтобы ставка не устаревала).
"""
import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rs_interfaces.msg import ExplorationBid
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rs_exploration import air_targets, auction
from rcl_interfaces.msg import ParameterDescriptor

FLY_STATES = ('READY', 'MOVING')
GONE_STATES = ('LANDING', 'LANDED', 'FAULT')   # дрон вышел из задачи не по нашей команде


class AirExplorer(Node):
    def __init__(self):
        super().__init__('air_explorer')
        P = self.declare_parameter
        P('robot', 'drone_1')
        P('air_robots', [], ParameterDescriptor(dynamic_typing=True))   # обязателен, задаёт launch
        P('region', [-11.0, -9.0, 11.0, 9.0])   # область поиска (xmin, ymin, xmax, ymax), как у наземных
        P('cell_m', 2.0)              # размер крупной клетки, м
        P('unseen_frac', 0.3)         # доля неизвестных мелких клеток, выше которой клетка "не видна"
        P('plan_period', 1.0)
        P('num_bids', 6)
        P('assign_radius', 4.0)       # цели двух дронов не ближе этого друг к другу
        P('other_goal_penalty', 6.0)  # штраф цели рядом с целью другого дрона, м
        P('hysteresis', 1.5)          # бонус цене текущей цели, м
        P('bid_ttl', 5.0)
        P('max_retries', 2)           # повторов клетки после ABORTED
        P('done_cycles', 3)
        P('speed', 2.0)               # крейсерская скорость дрона, м/с (для тайм-аута цели)
        P('home_tol', 1.0)            # в этом радиусе от точки старта идём на посадку, м
        P('auction_delay', 0.3)

        self.me = self.get_parameter('robot').value
        self.air = list(self.get_parameter('air_robots').value or [])
        if not self.air:
            raise RuntimeError('параметр air_robots пуст: список дронов задаёт launch (explore.launch.py), запасного списка нет')
        self.region = [float(v) for v in self.get_parameter('region').value]
        self.cell_m = float(self.get_parameter('cell_m').value)
        self.R = float(self.get_parameter('assign_radius').value)
        self.K = int(self.get_parameter('num_bids').value)
        self.speed = float(self.get_parameter('speed').value)

        self.map_msg = None
        self.pose = None
        self.state = None
        self.home = None            # (x, y) где впервые был READY
        self.book = air_targets.CellBook(int(self.get_parameter('max_retries').value))
        self.tracker = air_targets.DoneTracker(int(self.get_parameter('done_cycles').value))
        self.table = {}             # robot -> (получено, сообщение)
        self.seq = 0
        self.round = 0
        self.goal = None            # (x, y)
        self.goal_key = None        # ключ клетки или 'home'
        self.goal_handle = None
        self.goal_id = 0
        self.goal_t0 = 0.0
        self.goal_tmo = 0.0
        self.top = []
        self.phase = 'explore'      # explore | homing | landing | done
        self.done = False
        self.home_fails = 0
        self.land_busy = False
        self.land_logged = False

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map_air', self._on_map, map_qos)
        self.create_subscription(Odometry, 'pose', self._on_odom, 10)
        self.create_subscription(String, 'state', self._on_state, map_qos)
        self.create_subscription(ExplorationBid, '/exploration/air_bids', self._on_bid, 20)
        self.bid_pub = self.create_publisher(ExplorationBid, '/exploration/air_bids', 20)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.land_cli = self.create_client(Trigger, 'land')
        self.create_timer(float(self.get_parameter('plan_period').value), self._cycle)
        self.get_logger().info(f'air_explorer {self.me} запущен, область {self.region}, клетка {self.cell_m} м')

    # --- вспомогательное ----------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_map(self, msg):
        self.map_msg = msg

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def _on_state(self, msg):
        self.state = msg.data
        if msg.data == 'READY' and self.home is None and self.pose is not None:
            self.home = self.pose
            self.get_logger().info(f'точка старта ({self.home[0]:.1f}, {self.home[1]:.1f})')

    def _on_bid(self, msg):
        if msg.robot_id != self.me and msg.robot_id in self.air:
            self.table[msg.robot_id] = (self._now(), msg)

    # --- основной цикл ------------------------------------------------------
    def _cycle(self):
        if self.map_msg is None or self.pose is None or self.state is None:
            return
        if self.home is None and self.state == 'READY':
            self.home = self.pose
            self.get_logger().info(f'точка старта ({self.home[0]:.1f}, {self.home[1]:.1f})')
        now = self._now()
        ttl = float(self.get_parameter('bid_ttl').value)
        for rid in [k for k, (t, _) in self.table.items() if now - t > ttl]:
            del self.table[rid]
            self.get_logger().warn(f'ставки {rid} устарели: считаем выбывшим')

        if self.phase == 'explore' and self.state in GONE_STATES:
            self.get_logger().warn(f'дрон в состоянии {self.state}: выходит из задачи, считаем done')
            self.phase = 'done'
            self.done = True
            self.goal = None
        if self.phase == 'done':
            self._publish_bid([])
            return
        if self.phase == 'landing':
            self._try_land()
            self._publish_bid([])
            return
        if self.phase == 'homing':
            self._cycle_home(now)
            self._publish_bid([])
            return

        m = self.map_msg
        data = np.asarray(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
        keys = air_targets.unseen_cells(data, m.info.resolution, m.info.origin.position.x,
                                        m.info.origin.position.y, self.region, self.cell_m,
                                        float(self.get_parameter('unseen_frac').value))
        skip = self.book.skip()
        cand_keys = [k for k in keys if k not in skip]
        self._check_goal(cand_keys, now)
        if self.state not in FLY_STATES:
            self._publish_bid([])   # взлёт ещё идёт: ставок нет, но жив
            return
        gp = self.get_parameter
        others = [(msg.current_goal.x, msg.current_goal.y) for _, msg in self.table.values() if msg.has_goal]
        cands = air_targets.rank_cells(
            cand_keys, self.region, self.cell_m, self.pose, skip=skip,
            goal_key=self.goal_key if self.goal is not None else None,
            hysteresis=float(gp('hysteresis').value), others=others, assign_radius=self.R,
            other_penalty=float(gp('other_goal_penalty').value))
        if self.tracker.update(len(cands), self.goal is not None):
            self._finish_explore()
            self._publish_bid([])
            return
        top = [(float(np.float32(c)), x, y, k) for c, x, y, k in cands[:self.K]]
        self.top = top
        self._publish_bid(top)
        holder = []

        def run_auction():
            holder[0].cancel()
            self.destroy_timer(holder[0])
            self._auction()
        holder.append(self.create_timer(float(gp('auction_delay').value), run_auction))

    def _auction(self):
        if self.phase != 'explore':
            return
        table = {self.me: [(c, x, y, '') for c, x, y, _ in self.top]}
        for rid, (_, msg) in self.table.items():
            table[rid] = [(float(c), p.x, p.y, '') for c, p in zip(msg.costs, msg.targets)]
        res = auction.assign(table, self.R)
        self.round += 1
        mine = res.get(self.me)
        if mine is None:
            return   # нет назначения: продолжаем текущую цель, иначе ждём
        x, y, _ = mine
        key = air_targets.cell_key(x, y, self.region, self.cell_m)
        if self.goal is not None and key == self.goal_key:
            return
        if self.state not in FLY_STATES or not self.nav.server_is_ready():
            return
        self._send_goal(x, y, key)

    def _check_goal(self, cand_keys, now):
        """Цель уже просмотрена по пути: отмена. Тайм-аут: неудача клетки."""
        if self.goal is None:
            return
        if now - self.goal_t0 > self.goal_tmo:
            key = self.goal_key
            self._cancel_goal()
            self.get_logger().warn(f'тайм-аут цели {key}')
            self._abort_cell(key)
        elif self.goal_key not in cand_keys:
            self.get_logger().info(f'клетка {self.goal_key} уже просмотрена: отмена цели')
            self._cancel_goal()

    # --- завершение -----------------------------------------------------------
    def _finish_explore(self):
        self.get_logger().info(f'кандидатов нет: возврат в точку старта {self.home}, посадка')
        self.phase = 'homing'
        self._cycle_home(self._now())

    def _cycle_home(self, now):
        home = self.home if self.home is not None else self.pose
        d = math.hypot(home[0] - self.pose[0], home[1] - self.pose[1])
        if d < float(self.get_parameter('home_tol').value) or self.home_fails > 2:
            self._cancel_goal()
            self.phase = 'landing'
            self._try_land()
            return
        if self.goal is not None:
            if now - self.goal_t0 > self.goal_tmo:
                self._cancel_goal()
                self.home_fails += 1
            return
        if self.state in FLY_STATES and self.nav.server_is_ready():
            self._send_goal(home[0], home[1], 'home')

    def _try_land(self):
        if self.land_busy:
            return
        if self.state in ('LANDING', 'LANDED'):
            self._mark_done()
            return
        if not self.land_cli.service_is_ready():
            return
        self.land_busy = True
        fut = self.land_cli.call_async(Trigger.Request())

        def done(f):
            self.land_busy = False
            r = f.result()
            if r is not None and r.success:
                self._mark_done()
            else:
                self.get_logger().warn(f'land отказал: {getattr(r, "message", "нет ответа")}')
        fut.add_done_callback(done)

    def _mark_done(self):
        if not self.done:
            self.done = True
            self.phase = 'done'
            self.get_logger().info(f'посадка запрошена, done: облетено {len(self.book.visited)}, '
                                   f'провалено {len(self.book.failed)} клеток')

    # --- действие NavigateToPose ----------------------------------------------
    def _send_goal(self, x, y, key):
        self.goal_id += 1
        gid = self.goal_id
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'world'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(x)
        g.pose.pose.position.y = float(y)
        g.pose.pose.orientation.w = 1.0
        dist = math.hypot(x - self.pose[0], y - self.pose[1])
        self.goal, self.goal_key, self.goal_handle = (x, y), key, None
        self.goal_t0 = self._now()
        self.goal_tmo = air_targets.goal_timeout(dist, self.speed)
        self.get_logger().info(f'цель #{gid}: ({x:.1f}, {y:.1f}) клетка {key}, {dist:.1f} м')
        self.nav.send_goal_async(g).add_done_callback(lambda f, gid=gid: self._on_accept(f, gid))

    def _on_accept(self, fut, gid):
        gh = fut.result()
        if gid != self.goal_id:
            if gh.accepted:
                gh.cancel_goal_async()
            return
        if not gh.accepted:
            self.get_logger().warn(f'цель #{gid} отклонена дроном')
            key = self.goal_key
            self.goal, self.goal_key = None, None
            self._abort_cell(key)
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f, gid=gid: self._on_result(f, gid))

    def _on_result(self, fut, gid):
        if gid != self.goal_id or self.goal is None:
            return   # устаревший или отменённый нами результат
        status = fut.result().status
        key = self.goal_key
        self.goal, self.goal_key, self.goal_handle = None, None, None
        self.goal_id += 1
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'цель #{gid} достигнута, клетка {key} облетена')
            if key == 'home':
                self.phase = 'landing'
                self._try_land()
            else:
                self.book.on_success(key)
        else:
            self.get_logger().warn(f'цель #{gid}: статус {status}')
            self._abort_cell(key)

    def _abort_cell(self, key):
        if key == 'home':
            self.home_fails += 1
            return
        if key is None:
            return
        if not self.book.on_abort(key):
            self.get_logger().warn(f'клетка {key} провалена после {self.book.aborts[key]} неудач')

    def _cancel_goal(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal, self.goal_key, self.goal_handle = None, None, None
        self.goal_id += 1   # результат отменённой цели игнорируем

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
            msg.current_goal = Point(x=float(self.goal[0]), y=float(self.goal[1]))
        msg.targets = [Point(x=float(x), y=float(y)) for _, x, y, _ in top]
        msg.costs = [float(c) for c, *_ in top]
        msg.target_ids = [''] * len(top)
        msg.open_candidates = 0
        msg.gave_up = []
        msg.done = self.done
        self.bid_pub.publish(msg)


def main():
    rclpy.init()
    node = AirExplorer()
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

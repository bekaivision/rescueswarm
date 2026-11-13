"""Узел drone_adapter: один на дрон. Контракт дрона (как у наземного робота, но без Nav2).

Что делает:
  - сам взлетает после старта (TAKEOFF -> READY) на высоту altitude и держит позицию;
  - действие navigate_to_pose (nav2_msgs/NavigateToPose): цель x, y в кадре world, высота фиксирована;
    SUCCEEDED в пределах 0.5 м по горизонтали на малой скорости, ABORTED по таймауту
    (расстояние / скорость * 3 + 20 с) или если дрон не в воздухе, отмена поддерживается,
    новая цель вытесняет старую (старая получает ABORTED);
  - сервис land (std_srvs/Trigger): посадка на месте, состояния LANDING -> LANDED;
  - топик state (std_msgs/String, RELIABLE + TRANSIENT_LOCAL): INIT, TAKEOFF, READY, MOVING,
    LANDING, LANDED, FAULT;
  - TF world -> <robot>/base_link и топик pose (Odometry) из позы бэкенда.
Имена относительные: узел запускается в namespace /<robot>.
Параметры: robot, backend (light | px4), altitude (м над полом), speed (м/с, 2.0), use_sim_time.
Полёт спрятан в классе FlightInterface (flight.py): логика ниже от бэкенда не зависит.
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from rs_drone import controller as ctl
from rs_drone.flight import FlightUnavailable, make_flight

INIT, TAKEOFF, READY, MOVING, LANDING, LANDED, FAULT = (
    'INIT', 'TAKEOFF', 'READY', 'MOVING', 'LANDING', 'LANDED', 'FAULT')
AIRBORNE = (READY, MOVING)


class GoalCtx:
    """Одна принятая цель. outcome ставит цикл управления, а execute_callback ждёт done."""

    def __init__(self, handle, xy, timeout, t0):
        self.handle = handle
        self.xy = xy
        self.timeout = timeout
        self.t0 = t0
        self.outcome = None      # 'succeeded' | 'aborted' | 'canceled'
        self.message = ''
        self.done = threading.Event()

    def finish(self, outcome, message=''):
        if not self.done.is_set():
            self.outcome, self.message = outcome, message
            self.done.set()


class DroneAdapter(Node):
    def __init__(self):
        super().__init__('drone_adapter')
        self.declare_parameter('robot', 'drone_1')
        self.declare_parameter('backend', 'light')
        self.declare_parameter('altitude', 4.5)
        self.declare_parameter('speed', 2.0)
        self.declare_parameter('max_vz', 1.0)             # вертикальная скорость, м/с
        self.declare_parameter('max_accel_xy', 1.5)       # м/с^2
        self.declare_parameter('max_accel_z', 1.0)
        self.declare_parameter('hold_speed', 0.5)         # предел скорости при удержании позиции
        self.declare_parameter('xy_tol', 0.5)             # допуск цели по горизонтали, м
        self.declare_parameter('land_z', 0.14)            # высота центра дрона на земле, м
        self.declare_parameter('rate', 20.0)              # частота цикла управления (сим. время)
        self.declare_parameter('body_frame_cmd', True)    # light: плагин velocity-control читает скорость в системе корпуса
        # px4: точка старта в мире (начало локальной системы EKF), курс и номер системы MAVLink (экземпляр + 1)
        self.declare_parameter('origin', [0.0, 0.0, 0.0])
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('target_system', 2)
        self.robot = self.get_parameter('robot').value
        self.backend = self.get_parameter('backend').value
        self.altitude = float(self.get_parameter('altitude').value)
        self.speed = float(self.get_parameter('speed').value)
        self.max_vz = float(self.get_parameter('max_vz').value)
        self.a_xy = float(self.get_parameter('max_accel_xy').value)
        self.a_z = float(self.get_parameter('max_accel_z').value)
        self.hold_speed = float(self.get_parameter('hold_speed').value)
        self.xy_tol = float(self.get_parameter('xy_tol').value)
        self.land_z = float(self.get_parameter('land_z').value)
        self.dt_nom = 1.0 / float(self.get_parameter('rate').value)
        self.frame_child = f'{self.robot}/base_link'

        self.lock = threading.RLock()
        self.cbg = ReentrantCallbackGroup()
        self.state = INIT
        self._published = False
        self.pos = None                 # (x, y, z) последняя поза
        self.vel_est = ctl.VelocityEstimator()
        self.vel = (0.0, 0.0, 0.0)
        self.cmd = (0.0, 0.0, 0.0)      # последняя отданная команда скорости
        self.hold_xy = None
        self.ctx = None
        self.t_last = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, 'state', latched)
        self.pose_pub = self.create_publisher(Odometry, 'pose', 10)
        self.tf_br = TransformBroadcaster(self)
        self.create_service(Trigger, 'land', self._on_land, callback_group=self.cbg)
        self.action = ActionServer(
            self, NavigateToPose, 'navigate_to_pose',
            execute_callback=self._execute, goal_callback=self._on_goal,
            handle_accepted_callback=self._on_accepted, cancel_callback=self._on_cancel,
            callback_group=self.cbg)

        self.flight = None
        try:
            if self.backend == 'light':
                kw = {'body_frame_cmd': bool(self.get_parameter('body_frame_cmd').value)}
            else:
                kw = {'origin': [float(v) for v in self.get_parameter('origin').value],
                      'yaw': float(self.get_parameter('yaw').value),
                      'target_system': int(self.get_parameter('target_system').value)}
            self.flight = make_flight(self.backend, self, self._on_odom, **kw)
        except FlightUnavailable as e:
            self.get_logger().error(f'{self.robot}: {e}. Состояние FAULT, цели не принимаются')
            self._set_state(FAULT)
            return
        self._set_state(INIT)
        self.create_timer(self.dt_nom, self._tick, callback_group=self.cbg)
        self.get_logger().info(
            f'{self.robot}: drone_adapter, бэкенд {self.backend}, высота {self.altitude:.1f} м, скорость {self.speed:.1f} м/с')

    # ------------------------------------------------------------ состояние
    def _set_state(self, s):
        """Сменить состояние и опубликовать его (повторно то же состояние не публикуем)."""
        with self.lock:
            if s == self.state and self._published:
                return
            if s != self.state:
                self.get_logger().info(f'{self.robot}: состояние {self.state} -> {s}')
            self.state = s
            self._published = True
            self.state_pub.publish(String(data=s))
            if getattr(self, 'flight', None) is not None:
                self.flight.on_state(s)

    # ------------------------------------------------------------ поза, TF
    def _on_odom(self, msg):
        p = msg.pose.pose.position
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        with self.lock:
            self.pos = (p.x, p.y, p.z)
            self.vel = self.vel_est.update(t, self.pos)
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = 'world'
        out.child_frame_id = self.frame_child
        out.pose = msg.pose
        out.twist = msg.twist
        self.pose_pub.publish(out)
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'world'
        tf.child_frame_id = self.frame_child
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = p.x, p.y, p.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf_br.sendTransform(tf)

    # ------------------------------------------------------------ действие NavigateToPose
    def _on_goal(self, goal):
        with self.lock:
            ok = self.state in AIRBORNE
            st = self.state
        if not ok:
            self.get_logger().warn(f'{self.robot}: цель отклонена, состояние {st} (нужно READY или MOVING)')
            return GoalResponse.REJECT
        fid = goal.pose.header.frame_id
        if fid not in ('', 'world'):
            self.get_logger().warn(f"{self.robot}: кадр цели '{fid}', считаю его world")
        return GoalResponse.ACCEPT

    def _on_cancel(self, handle):
        return CancelResponse.ACCEPT

    def _on_accepted(self, handle):
        p = handle.request.pose.pose.position
        xy = (p.x, p.y)
        with self.lock:
            dist = math.hypot(xy[0] - self.pos[0], xy[1] - self.pos[1]) if self.pos else 0.0
            timeout = ctl.goal_timeout(dist, self.speed)
            if self.ctx is not None:
                self.ctx.finish('aborted', 'вытеснена новой целью')
            self.ctx = GoalCtx(handle, xy, timeout, self.get_clock().now().nanoseconds * 1e-9)
            self.get_logger().info(
                f'{self.robot}: цель ({xy[0]:.2f}, {xy[1]:.2f}), {dist:.1f} м, таймаут {timeout:.0f} с')
            self._set_state(MOVING)
        handle.execute()

    def _execute(self, handle):
        with self.lock:
            ctx = self.ctx if self.ctx is not None and self.ctx.handle is handle else None
        result = NavigateToPose.Result()
        if ctx is None:       # вытеснена до начала выполнения
            handle.abort()
            return result
        last_fb = 0.0
        while not ctx.done.is_set() and rclpy.ok():
            if handle.is_cancel_requested:
                ctx.finish('canceled', 'отмена')
                break
            now = time.monotonic()
            if now - last_fb > 0.5:
                last_fb = now
                fb = NavigateToPose.Feedback()
                with self.lock:
                    pos = self.pos
                if pos:
                    fb.current_pose.header.frame_id = 'world'
                    fb.current_pose.pose.position.x, fb.current_pose.pose.position.y = pos[0], pos[1]
                    fb.current_pose.pose.position.z = pos[2]
                    fb.distance_remaining = float(math.hypot(ctx.xy[0] - pos[0], ctx.xy[1] - pos[1]))
                handle.publish_feedback(fb)
            time.sleep(0.05)
        if not ctx.done.is_set():
            ctx.finish('aborted', 'узел остановлен')
        with self.lock:
            if ctx.outcome == 'canceled' and self.ctx is ctx:
                self._release_goal(ctx)
        if ctx.outcome == 'succeeded':
            handle.succeed()
        elif ctx.outcome == 'canceled':
            handle.canceled()
        else:
            handle.abort()
            if hasattr(result, 'error_code'):
                result.error_code = 9000
                result.error_msg = ctx.message
        self.get_logger().info(f'{self.robot}: цель завершена: {ctx.outcome} {ctx.message}')
        return result

    def _release_goal(self, ctx):
        """Цель закончилась: держим текущую точку, возвращаемся в READY (если ещё в воздухе)."""
        if self.ctx is ctx:
            self.ctx = None
        if self.pos is not None:
            self.hold_xy = (self.pos[0], self.pos[1])
        if self.state == MOVING:
            self._set_state(READY)

    # ------------------------------------------------------------ посадка
    def _on_land(self, _req, resp):
        with self.lock:
            st = self.state
            if st in (LANDING, LANDED):
                resp.success, resp.message = True, f'уже {st}'
                return resp
            if st in (INIT, FAULT) or self.pos is None:
                resp.success, resp.message = False, f'нельзя сесть из состояния {st}'
                return resp
            if self.ctx is not None:
                self.ctx.finish('aborted', 'посадка')
                self.ctx = None
            self.hold_xy = (self.pos[0], self.pos[1])
            self._set_state(LANDING)
        resp.success, resp.message = True, 'посадка'
        return resp

    # ------------------------------------------------------------ цикл управления
    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        with self.lock:
            dt = self.dt_nom if self.t_last is None else min(max(now - self.t_last, 1e-3), 0.2)
            self.t_last = now
            pos, st = self.pos, self.state
            if pos is None:
                return
            if self.hold_xy is None:
                self.hold_xy = (pos[0], pos[1])
            speed_xy = math.hypot(self.vel[0], self.vel[1])
            new_state = None
            target = (self.hold_xy[0], self.hold_xy[1], self.altitude)
            vmax = self.hold_speed
            if st == INIT:
                # PX4: взлёт только после готовности оценки позы и предполётных проверок
                if self.flight is not None and self.flight.ready():
                    new_state = TAKEOFF
            elif st == TAKEOFF:
                if ctl.at_altitude(pos[2], self.altitude, self.vel[2]):
                    new_state = READY
            elif st == MOVING and self.ctx is not None:
                c = self.ctx
                d = math.hypot(c.xy[0] - pos[0], c.xy[1] - pos[1])
                target = (c.xy[0], c.xy[1], self.altitude)
                vmax = self.speed
                if ctl.goal_reached(d, speed_xy, self.xy_tol):
                    self.hold_xy = c.xy
                    c.finish('succeeded')
                    self.ctx = None
                    new_state = READY
                elif now - c.t0 > c.timeout:
                    c.finish('aborted', 'таймаут')
                    self.hold_xy = (pos[0], pos[1])
                    self.ctx = None
                    new_state = READY
            elif st == LANDING:
                target = (self.hold_xy[0], self.hold_xy[1], self.land_z - 0.3)   # ниже земли: садимся до касания
                vmax = self.hold_speed
                if pos[2] <= self.land_z + 0.05 and abs(self.vel[2]) < 0.3:
                    new_state = LANDED
            elif st in (LANDED, FAULT):
                target = None
            if target is None:
                self.cmd = (0.0, 0.0, 0.0)
            else:
                vz_max = self.max_vz if st != LANDING else min(self.max_vz, 0.5)
                self.cmd = ctl.velocity_command(self.cmd, pos, target, vmax, vz_max, self.a_xy, self.a_z, dt)
            cmd = self.cmd
            if new_state is not None:
                self._set_state(new_state)
        if self.flight is not None:
            self.flight.send_velocity(*cmd)


def main():
    rclpy.init()
    node = DroneAdapter()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.flight is not None:
            try:
                node.flight.stop()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

"""Полётная часть дрона: тонкий слой между логикой адаптера и тем, что реально летает.

Логика (состояния, действие NavigateToPose, посадка) от бэкенда не зависит и работает только через
класс FlightInterface: он отдаёт позу (Odometry) через обратный вызов и принимает команду скорости.
LightFlight: Gazebo velocity-control, поза истинная. Px4Flight: PX4 SITL (offboard, поза от EKF).
"""
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from rs_drone import controller


class FlightUnavailable(RuntimeError):
    """Бэкенд нельзя использовать (не реализован или нет связи)."""


class FlightInterface:
    """Что нужно адаптеру от бэкенда."""

    def __init__(self, node, on_odom):
        self.node = node
        self.on_odom = on_odom      # вызывается с nav_msgs/Odometry (поза в кадре world)

    def send_velocity(self, vx, vy, vz):
        """Команда скорости в мировой системе, м/с."""
        raise NotImplementedError

    def stop(self):
        self.send_velocity(0.0, 0.0, 0.0)

    def ready(self):
        """Можно взлетать (PX4: оценка позы готова, предполётные проверки пройдены)."""
        return True

    def on_state(self, state):
        """Адаптер сменил состояние (PX4 по нему армит или дизармит)."""


class LightFlight(FlightInterface):
    """Лёгкий дрон: поза из /<robot>/gt_odom, скорость в /<robot>/cmd_vel (плагин velocity-control).

    Параметр body_frame_cmd: плагин понимает скорость в системе корпуса, а не в мировой
    (проверяется в смоук-тесте), тогда команда поворачивается обратно на курс дрона.
    """

    def __init__(self, node, on_odom, body_frame_cmd=False):
        super().__init__(node, on_odom)
        self.body_frame_cmd = body_frame_cmd
        self.yaw = 0.0
        self.pub = node.create_publisher(Twist, 'cmd_vel', 10)
        node.create_subscription(Odometry, 'gt_odom', self._odom, 10)

    def _odom(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = controller.yaw_from_quat(q.x, q.y, q.z, q.w)
        self.on_odom(msg)

    def send_velocity(self, vx, vy, vz):
        if self.body_frame_cmd:
            vx, vy = controller.world_to_body_xy(vx, vy, self.yaw)
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = float(vx), float(vy), float(vz)
        self.pub.publish(t)


class Px4Flight(FlightInterface):
    """PX4 SITL через uXRCE-DDS: режим offboard, уставки скорости, поза от EKF PX4.

    Топики PX4 в пространстве имён дрона (PX4_UXRCE_DDS_NS = <robot>): fmu/in/offboard_control_mode,
    fmu/in/trajectory_setpoint, fmu/in/vehicle_command; fmu/out/vehicle_odometry,
    fmu/out/vehicle_status_v1, fmu/out/vehicle_local_position_v1 (версии сообщений px4_msgs v1.17).
    Подписки только BEST_EFFORT (иначе данные от агента не приходят).
    Поза: оценка EKF в NED относительно точки старта -> мир ENU сдвигом на точку старта (origin).
    Это оценка, а не истина: истинная поза идёт отдельно в /<robot>/gt_odom (только для оценки).
    Арминг и offboard запрашиваются, только пока адаптер хочет быть в воздухе (TAKEOFF/READY/MOVING/
    LANDING); после LANDED - disarm.
    """

    OFFBOARD = 14            # VehicleStatus.NAVIGATION_STATE_OFFBOARD
    ARMED = 2                # VehicleStatus.ARMING_STATE_ARMED

    def __init__(self, node, on_odom, origin=(0.0, 0.0, 0.0), yaw=0.0, target_system=2, **_):
        super().__init__(node, on_odom)
        try:
            from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                                      VehicleLocalPosition, VehicleOdometry, VehicleStatus)
        except ImportError as e:
            raise FlightUnavailable(f'нет px4_msgs ({e}): соберите make deps && make build') from e
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

        from rs_drone import px4_frames as F
        self.F = F
        self.M = dict(ocm=OffboardControlMode, sp=TrajectorySetpoint, cmd=VehicleCommand)
        self.origin = tuple(origin)
        self.yaw_ned = F.yaw_enu_to_ned(yaw)
        self.target = int(target_system)
        self.v_enu = (0.0, 0.0, 0.0)
        self.want_air = False
        self.disarm_req = False
        self.armed = False
        self.nav = -1
        self.checks = False
        self.xy_valid = self.z_valid = False
        self.got_odom = False
        self.t_cmd = -1e9
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pub_ocm = node.create_publisher(OffboardControlMode, 'fmu/in/offboard_control_mode', 10)
        self.pub_sp = node.create_publisher(TrajectorySetpoint, 'fmu/in/trajectory_setpoint', 10)
        self.pub_cmd = node.create_publisher(VehicleCommand, 'fmu/in/vehicle_command', 10)
        node.create_subscription(VehicleOdometry, 'fmu/out/vehicle_odometry', self._odom, be)
        node.create_subscription(VehicleStatus, 'fmu/out/vehicle_status_v1', self._status, be)
        node.create_subscription(VehicleLocalPosition, 'fmu/out/vehicle_local_position_v1', self._lpos, be)
        node.get_logger().info(f'px4: цель target_system={self.target}, старт в мире {self.origin}')

    # ---- от PX4
    def _now_us(self):
        return self.node.get_clock().now().nanoseconds // 1000

    def _status(self, m):
        self.armed = m.arming_state == self.ARMED
        self.nav = m.nav_state
        self.checks = bool(m.pre_flight_checks_pass)

    def _lpos(self, m):
        self.xy_valid, self.z_valid = bool(m.xy_valid), bool(m.z_valid)

    def _odom(self, m):
        F = self.F
        if m.pose_frame != 1:       # 1 = NED; FRD-кадр (без GPS) не поддерживаем
            return
        self.got_odom = True
        # поля px4_msgs приходят как numpy.float32: сообщения ROS принимают только float
        x, y, z = F.ned_to_world(tuple(float(v) for v in m.position), self.origin)
        vx, vy, vz = (float(v) for v in F.vel_ned_to_enu(tuple(float(v) for v in m.velocity)))
        qw, qx, qy, qz = (float(v) for v in F.q_ned_frd_to_enu_flu(tuple(float(v) for v in m.q)))
        o = Odometry()
        # Время оценки: timestamp_sample PX4. В lockstep время PX4 = время симуляции (мкс), поэтому позу
        # штампуем моментом измерения, а не приходом сообщения: облако глубины ищет TF на своём штампе, и
        # задержка XRCE/DDS при наклоне дрона давала ошибку наклона и ложные препятствия по краям кадра.
        # Если время не похоже на сим. время (больше 0.5 с разницы), берём текущее.
        # Позиция EKF совпадает по времени с сенсорами Gazebo, только если EKF2_GPS_DELAY = 0 (px4_instance.sh):
        # при умолчании 110 мс EKF считает данные GPS старыми, и позиция уходит вперёд на 0.11 с * скорость.
        now = self.node.get_clock().now()
        ts = int(m.timestamp_sample) * 1000
        if abs(ts - now.nanoseconds) >= 500_000_000:
            ts = now.nanoseconds
        o.header.stamp.sec, o.header.stamp.nanosec = ts // 1_000_000_000, ts % 1_000_000_000
        o.header.frame_id = 'world'
        p = o.pose.pose
        p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
        p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z = qw, qx, qy, qz
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = vx, vy, vz
        self.on_odom(o)

    # ---- интерфейс адаптера
    def ready(self):
        return self.got_odom and self.checks and self.xy_valid and self.z_valid

    def on_state(self, state):
        self.want_air = state in ('TAKEOFF', 'READY', 'MOVING', 'LANDING')
        self.disarm_req = state in ('LANDED', 'FAULT')

    def send_velocity(self, vx, vy, vz):
        """Вызывается циклом адаптера (20 Гц сим. времени): стримим offboard и при нужде армим."""
        self.v_enu = (vx, vy, vz)
        now = self._now_us()
        nan = float('nan')
        ocm = self.M['ocm']()
        ocm.timestamp = now
        ocm.position, ocm.velocity = False, True
        self.pub_ocm.publish(ocm)
        sp = self.M['sp']()
        sp.timestamp = now
        sp.position = [nan, nan, nan]
        sp.velocity = [float(v) for v in self.F.vel_enu_to_ned(self.v_enu)]
        sp.acceleration = [nan, nan, nan]
        sp.jerk = [nan, nan, nan]
        sp.yaw = float(self.yaw_ned)
        sp.yawspeed = nan
        self.pub_sp.publish(sp)
        if now - self.t_cmd < 1_000_000:      # команды не чаще раза в секунду
            return
        if self.want_air and not (self.armed and self.nav == self.OFFBOARD):
            self.t_cmd = now
            self._command(176, 1.0, 6.0)     # DO_SET_MODE: custom, offboard
            self._command(400, 1.0)          # ARM
        elif self.disarm_req and self.armed:
            self.t_cmd = now
            self._command(400, 0.0)          # DISARM

    def _command(self, command, p1=0.0, p2=0.0):
        c = self.M['cmd']()
        c.timestamp = self._now_us()
        c.command = command
        c.param1, c.param2 = float(p1), float(p2)
        c.target_system, c.target_component = self.target, 1
        c.source_system, c.source_component = 1, 1
        c.from_external = True
        self.pub_cmd.publish(c)


def make_flight(backend, node, on_odom, **kw):
    if backend == 'light':
        return LightFlight(node, on_odom, **kw)
    if backend == 'px4':
        return Px4Flight(node, on_odom, **kw)
    raise FlightUnavailable(f"неизвестный бэкенд '{backend}', нужно light или px4")

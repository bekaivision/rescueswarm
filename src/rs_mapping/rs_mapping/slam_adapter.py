"""Узел slam_adapter: мост между mrg_slam и остальным стеком (Фаза 4).

Одометрия для mrg_slam (слияние колёс и scan matching). Сырой выход scan matching
/<r>/scan_matching_odometry/raw_odom (ремап в slam.launch.py) нельзя отдавать графу как есть: вдоль длинной прямой
стены (коридор) лидар не измеряет смещение вдоль неё, и scan matching там стоит на месте или гуляет (опыты Фазы 4:
в коридорах он почти не двигался, в комнатах работал с ошибкой 0.1-0.2 м на 60 м). Колёсный шаг в симуляторе точен,
но проскальзывает при упоре в препятствие, а при раскрутке колёс на месте теряет и курс. Поэтому шаг складывается так:
  * поворот: по гироскопу (/<r>/imu, интеграл угловой скорости между штампами сканов): он не проскальзывает и не зависит
    от геометрии. Без IMU (use_gyro=false): по scan matching, если он не расходится с колёсами больше max_yaw_diff за шаг,
    иначе по колёсам;
  * перемещение: колёсный шаг плюс поправка (шаг scan matching - колёсный шаг), но только вдоль тех направлений, которые
    геометрия кадра реально ограничивает. Это сообщает slam_cloud_crop в /<r>/slam_degeneracy: матрица I = сумма n*n^T
    по нормалям стен (2x2). По её собственным значениям lam_k поправка берётся с весом 1 - exp(-(lam_k/lam0)^2)
    (lam0 = deg_lambda0): в комнатах (оба lam порядка 1000) верим scan matching, в коридоре (lam вдоль стены порядка 6)
    колёсам. Поправка больше jump_abs + jump_rel * |колёсный шаг| считается ложным совмещением и отбрасывается;
    меньше dead_abs + dead_rel * |колёсный шаг| считается шумом scan matching и тоже отбрасывается (колёсный шаг гладкий,
    а шум scan matching за много шагов копится в дрейф). Остаётся случай, когда колёса явно врут (упор в стену, раскрутка
    колёс): тогда верим лидару.
Результат публикуется как /<r>/scan_matching_odometry/odom (его читает mrg_slam). Ошибки накопления одометрии
исправляют граф mrg_slam: замыкания петель и слияние графов роботов (замыкания по умолчанию выключены, см. slam.launch.py).
Дальше по двум потокам mrg_slam (без TF):
  /<r>/mrg_slam/odom2map                T_map<-slam_odom (раз в несколько секунд, после оптимизации графа)
  /<r>/scan_matching_odometry/odom      T_slam_odom<-base (слитая одометрия, 10 Гц)
считается поза базы в мире: T_world_base = T_world_map * odom2map * odom.
Привязка: карта mrg_slam совпадает с world, потому что init_pose каждого экземпляра равен точке
старта робота в world (из ground_truth.yaml). Поэтому T_world_map = единичная, и все роботы
лежат в одной общей карте (то, что получилось после обмена графами, уже в этой же системе).
Дальше строит поправку и публикует TF world -> <r>/odom:
  T_world_odom = T_world_base_slam * inv(T_odom_base_wheel),
где T_odom_base_wheel - колёсная одометрия из TF на штамп скана (интерполяция в буфере TF),
штампы не смешиваются. Поправку раз в 1/rate секунд заново штампуем текущим временем.
До первой позы SLAM публикуется поза старта (колёсная одометрия начинается в точке старта).
Все скачки поправки (замыкание петель, слияние графов) пишутся в jumps.csv в out_dir; скачки
не сглаживаются. В slam_trace.csv на каждый скан: поза SLAM, чисто колёсная поза, сырой scan matching и шаги
(scan matching, колёса, слитый) вместе с I, чтобы правила слияния можно было перебрать офлайн.
Параметры: robots, poses ("x,y,yaw;x,y,yaw;..." как в robots.launch.py), rate, out_dir, deg_lambda0 (200),
base_z (0.08 м, высота base_link над полом), jump_abs (0.05 м), jump_rel (0.3), dead_abs (0.005 м), dead_rel (0.2), use_gyro (true), max_yaw_diff (0.3 рад за шаг).
"""
import csv
import math
import os
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from rcl_interfaces.msg import ParameterDescriptor


def quat_to_mat(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 0 else 0.0
    xs, ys, zs = x * s, y * s, z * s
    return np.array([
        [1 - (y * ys + z * zs), x * ys - w * zs, x * zs + w * ys],
        [x * ys + w * zs, 1 - (x * xs + z * zs), y * zs - w * xs],
        [x * zs - w * ys, y * zs + w * xs, 1 - (x * xs + y * ys)]])


def mat_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(R[i, i] - R[j, j] - R[k, k] + 1.0) * 2
    q = [0.0] * 4
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


def pose_to_mat(t, q):
    T = np.eye(4)
    T[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w)
    T[:3, 3] = (t.x, t.y, t.z)
    return T


def inv(T):
    R = T[:3, :3].T
    out = np.eye(4)
    out[:3, :3] = R
    out[:3, 3] = -R @ T[:3, 3]
    return out


def yaw_of(T):
    return math.atan2(T[1, 0], T[0, 0])


def from_xyyaw(x, y, yaw):
    T = np.eye(4)
    c, s = math.cos(yaw), math.sin(yaw)
    T[:2, :2] = [[c, -s], [s, c]]
    T[:2, 3] = (x, y)
    return T


def world_odom(T_wb, T_ob, base_z):
    """T_world_odom = T_world_base * inv(T_odom_base), z берём не из SLAM: пол плоский, высота базы известна."""
    T = T_wb @ inv(T_ob)
    T[2, 3] = base_z
    return T


class SlamAdapter(Node):
    def __init__(self):
        super().__init__('slam_adapter')
        self.declare_parameter('robots', [], ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('poses', '')
        self.declare_parameter('rate', 20.0)
        # TF world -> <r>/odom штампуем на tf_future_s вперёд (как AMCL): при низком RTF (3 робота + 2 дрона, RTF 0.3-0.5)
        # последняя запись отстаёт на 0.05-0.3 с, и Nav2 (RPPPathHandler.transformPose) падает с "extrapolation into the future",
        # цель осмотра отменяется со статусом 6 (seed 1 dev: 10 из 10 отказов осмотра). 0 = как раньше.
        self.declare_parameter('tf_future_s', 0.3)
        self.declare_parameter('out_dir', '')
        self.declare_parameter('jump_abs', 0.05)
        self.declare_parameter('jump_rel', 0.3)
        self.declare_parameter('max_yaw_diff', 0.3)
        self.declare_parameter('dead_abs', 0.005)    # поправку меньше dead_abs + dead_rel * |колёсный шаг| не применяем (шум)
        self.declare_parameter('dead_rel', 0.2)
        self.declare_parameter('use_gyro', True)
        # Высота base_link над полом (радиус колеса). Колёсная одометрия плоская (z = 0), а истинная поза в gt-режиме
        # несёт настоящую высоту: без этой добавки все точки лидара в world оказывались на ~0.08 м ниже, и высота
        # предметов по лидару занижалась (люди 0.32 -> 0.23 м, коробки 0.54 -> 0.45 м), ловушки проходили по порогу 0.4 м.
        self.declare_parameter('base_z', 0.08)
        self.base_z = float(self.get_parameter('base_z').value)
        self.dead_abs = float(self.get_parameter('dead_abs').value)
        self.dead_rel = float(self.get_parameter('dead_rel').value)
        self.use_gyro = bool(self.get_parameter('use_gyro').value)
        self.declare_parameter('deg_lambda0', 200.0)       # сумма n*n^T, при которой scan matching верим наполовину
        self.jump_abs = float(self.get_parameter('jump_abs').value)
        self.jump_rel = float(self.get_parameter('jump_rel').value)
        self.max_yaw_diff = float(self.get_parameter('max_yaw_diff').value)
        self.lam0 = float(self.get_parameter('deg_lambda0').value)
        self.robots = list(self.get_parameter('robots').value or [])
        if not self.robots:
            raise RuntimeError('параметр robots пуст: список роботов задаёт launch (explore.launch.py / slam.launch.py), запасного списка нет')
        poses = [tuple(float(v) for v in p.split(',')) for p in self.get_parameter('poses').value.split(';') if p.strip()]
        if len(poses) < len(self.robots):
            raise RuntimeError(f'poses: задано {len(poses)} поз, нужно {len(self.robots)}')
        self.spawn = {r: from_xyyaw(*poses[i]) for i, r in enumerate(self.robots)}
        for T in self.spawn.values():
            T[2, 3] = self.base_z
        # состояние на робота
        self.t_wo = {r: self.spawn[r].copy() for r in self.robots}    # world -> odom (публикуем)
        self.o2m = {r: self.spawn[r].copy() for r in self.robots}     # map <- slam_odom; до первого сообщения = старт
        self.last_so = {r: np.eye(4) for r in self.robots}             # последняя поза скана (slam_odom <- base)
        self.pending = {r: deque(maxlen=50) for r in self.robots}      # (stamp, T_so)
        self.n_o2m = {r: 0 for r in self.robots}
        self.n_scan = {r: 0 for r in self.robots}
        self.n_fail = {r: 0 for r in self.robots}
        self.n_clip = {r: 0 for r in self.robots}      # шагов без оценки вырожденности (взят колёсный шаг)
        self.n_jump = {r: 0 for r in self.robots}       # шагов, где scan matching разошёлся с колёсами больше допуска
        self.n_yaw_wheel = {r: 0 for r in self.robots}  # шагов, где поворот взят у колёс
        self.prev_raw = {r: None for r in self.robots}  # (T_so_raw, T_ob_wheel) предыдущего скана
        self.fused = {r: np.eye(4) for r in self.robots}
        self.gyro = {r: deque(maxlen=400) for r in self.robots}         # (штамп нс, накопленный угол по гироскопу)
        self.gyro_last = {r: None for r in self.robots}                 # (штамп нс, wz)
        self.deg = {r: {} for r in self.robots}                         # штамп (нс) -> (Ixx, Iyy, Ixy)
        self.last_step = {r: None for r in self.robots}                 # для журнала: (dS, dW, I, шаг слитый)
        self.fused_pub = {r: self.create_publisher(Odometry, f'/{r}/scan_matching_odometry/odom', 32) for r in self.robots}
        self.slam_active = {r: False for r in self.robots}
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self, spin_thread=True)
        self.br = TransformBroadcaster(self)
        out_dir = self.get_parameter('out_dir').value
        self.csv = None
        self.trace = None
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
            self.csv_f = open(os.path.join(out_dir, 'jumps.csv'), 'w', newline='')
            self.csv = csv.writer(self.csv_f)
            self.csv.writerow(['t_sim', 'robot', 'n_odom2map', 'jump_xy_m', 'jump_yaw_deg', 'odom2map_x', 'odom2map_y', 'odom2map_yaw_deg'])
            self.trace_f = open(os.path.join(out_dir, 'slam_trace.csv'), 'w', newline='')
            self.trace = csv.writer(self.trace_f)
            self.trace.writerow(['t_sim', 'robot', 'lag_s', 'slam_x', 'slam_y', 'slam_yaw_deg', 'wheel_x', 'wheel_y', 'wheel_yaw_deg', 'raw_sm_x', 'raw_sm_y',
                                 'dS_x', 'dS_y', 'dS_yaw_deg', 'dW_x', 'dW_y', 'dW_yaw_deg', 'Ixx', 'Iyy', 'Ixy', 'f_x', 'f_y', 'f_yaw_deg', 'dG_yaw_deg'])
        for r in self.robots:
            self.create_subscription(TransformStamped, f'/{r}/mrg_slam/odom2map', lambda m, r=r: self.on_o2m(r, m), 10)
            self.create_subscription(Odometry, f'/{r}/scan_matching_odometry/raw_odom', lambda m, r=r: self.on_scan(r, m), 50)
            self.create_subscription(Imu, f'/{r}/imu', lambda m, r=r: self.on_imu(r, m), 100)
            self.create_subscription(Vector3Stamped, f'/{r}/slam_degeneracy', lambda m, r=r: self.on_deg(r, m), 50)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self.publish_tf)
        self.create_timer(0.02, self.tick)   # дожидаемся колёсного TF для сканов, пришедших раньше него
        self.create_timer(30.0, self.report)
        self.get_logger().info(f'slam_adapter: роботы {self.robots}, привязка карты mrg_slam к world по позам старта')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_o2m(self, r, msg):
        new = pose_to_mat(msg.transform.translation, msg.transform.rotation)
        old = self.o2m[r]
        # скачок: насколько сдвинулась бы текущая поза робота от смены поправки
        d = inv(old @ self.last_so[r]) @ (new @ self.last_so[r])
        jump_xy = float(np.hypot(d[0, 3], d[1, 3]))
        jump_yaw = math.degrees(yaw_of(d))
        self.o2m[r] = new
        self.n_o2m[r] += 1
        if self.n_o2m[r] == 1:
            self.get_logger().info(f'{r}: первая odom2map от mrg_slam (x={new[0, 3]:.2f} y={new[1, 3]:.2f} yaw={math.degrees(yaw_of(new)):.1f})')
        if self.csv:
            self.csv.writerow([f'{self.now_s():.2f}', r, self.n_o2m[r], f'{jump_xy:.4f}', f'{jump_yaw:.3f}',
                               f'{new[0, 3]:.4f}', f'{new[1, 3]:.4f}', f'{math.degrees(yaw_of(new)):.3f}'])
            self.csv_f.flush()
        if jump_xy > 0.3:
            self.get_logger().info(f'{r}: скачок поправки SLAM {jump_xy:.2f} м, {jump_yaw:.1f} град (замыкание петли или слияние графов)')

    def on_imu(self, r, msg):
        """Курс по гироскопу: интеграл угловой скорости вокруг z (трапеции по штампам сообщений)."""
        t = Time.from_msg(msg.header.stamp).nanoseconds
        wz = msg.angular_velocity.z
        last = self.gyro_last[r]
        self.gyro_last[r] = (t, wz)
        h = self.gyro[r]
        if last is None:
            h.append((t, 0.0))
            return
        dt = (t - last[0]) * 1e-9
        if 0.0 < dt < 0.5:
            h.append((t, h[-1][1] + 0.5 * (wz + last[1]) * dt))

    def gyro_at(self, r, t_ns):
        """Угол по гироскопу на штамп t_ns (линейная интерполяция); None, если данных на этот штамп ещё нет."""
        h = self.gyro[r]
        if len(h) < 2 or h[-1][0] < t_ns or h[0][0] > t_ns:
            return None
        for (t0, a0), (t1, a1) in zip(reversed(list(h)[:-1]), reversed(list(h)[1:])):
            if t0 <= t_ns <= t1:
                return a0 + (a1 - a0) * (t_ns - t0) / max(t1 - t0, 1)
        return None

    def on_deg(self, r, msg):
        d = self.deg[r]
        d[Time.from_msg(msg.header.stamp).nanoseconds] = (msg.vector.x, msg.vector.y, msg.vector.z)
        while len(d) > 100:
            d.pop(next(iter(d)))

    def on_scan(self, r, msg):
        self.n_scan[r] += 1
        self.pending[r].append((Time.from_msg(msg.header.stamp), msg))
        self.process(r)

    def fuse(self, r, T_raw, T_ob, info=None, theta=None):
        """Слитая поза скана в кадре slam_odom по сырой (scan matching) и колёсной одометрии."""
        if self.prev_raw[r] is None:
            self.prev_raw[r] = (T_raw, T_ob, theta)
            return self.fused[r]
        T_raw0, T_ob0, theta0 = self.prev_raw[r]
        self.prev_raw[r] = (T_raw, T_ob, theta)
        dS = inv(T_raw0) @ T_raw      # шаг по scan matching (в кадре базы)
        dW = inv(T_ob0) @ T_ob        # шаг по колёсам
        yaw_s, yaw_w = yaw_of(dS), yaw_of(dW)
        diff = (yaw_s - yaw_w + math.pi) % (2 * math.pi) - math.pi
        dg = float('nan')
        if theta is not None and theta0 is not None:
            dg = theta - theta0
            yaw = dg                 # гироскоп не проскальзывает и не зависит от геометрии
        elif abs(diff) > self.max_yaw_diff:
            yaw = yaw_w
            self.n_yaw_wheel[r] += 1
        else:
            yaw = yaw_s
        w, sm = dW[:2, 3], dS[:2, 3]
        nw = float(np.hypot(*w))
        xy = w
        if info is not None:
            lam, V = np.linalg.eigh(np.array([[info[0], info[2]], [info[2], info[1]]]))
            corr = np.zeros(2)
            for k in range(2):
                wk = 1.0 - math.exp(-(lam[k] / self.lam0) ** 2)    # резкий переход: слабое направление ~0, сильное ровно ~1
                corr += wk * V[:, k] * float(V[:, k] @ (sm - w))
            nc = float(np.hypot(*corr))
            if nc > self.jump_abs + self.jump_rel * nw:
                self.n_jump[r] += 1    # scan matching "прыгнул": верим колёсам
            elif nc > self.dead_abs + self.dead_rel * nw:
                xy = w + corr          # колёса явно расходятся с лидаром (проскальзывание, упор): верим лидару
            # иначе расхождение в пределах шума scan matching: оставляем колёсный шаг (он гладкий)
        else:
            self.n_clip[r] += 1    # нет оценки вырожденности для этого скана: колёса
        d = from_xyyaw(xy[0], xy[1], yaw)
        self.last_step[r] = (dS, dW, info, xy, yaw, dg)
        self.fused[r] = self.fused[r] @ d
        return self.fused[r]

    def process(self, r):
        q = self.pending[r]
        while q:
            stamp, msg = q[0]
            try:
                tf = self.tf_buffer.lookup_transform(f'{r}/odom', f'{r}/base_link', stamp)
            except TransformException:
                # колёсной одометрии на этот штамп ещё нет (или уже нет): ждём, но не вечно
                if (self.get_clock().now() - stamp).nanoseconds * 1e-9 > 1.0:
                    q.popleft()
                    self.n_fail[r] += 1
                    continue
                break
            info = self.deg[r].get(stamp.nanoseconds)
            if info is None and (self.get_clock().now() - stamp).nanoseconds * 1e-9 < 0.5:
                break    # нормали этого скана ещё не пришли
            theta = self.gyro_at(r, stamp.nanoseconds) if self.use_gyro else None
            if self.use_gyro and theta is None and self.gyro[r] and (self.get_clock().now() - stamp).nanoseconds * 1e-9 < 0.5:
                break    # гироскоп ещё не дошёл до штампа скана
            q.popleft()
            T_ob = pose_to_mat(tf.transform.translation, tf.transform.rotation)
            T_raw = pose_to_mat(msg.pose.pose.position, msg.pose.pose.orientation)
            T_so = self.fuse(r, T_raw, T_ob, info, theta)
            self.last_so[r] = T_so
            out = Odometry()
            out.header = msg.header
            out.child_frame_id = msg.child_frame_id
            out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(float, T_so[:3, 3])
            qx, qy, qz, qw = mat_to_quat(T_so[:3, :3])
            out.pose.pose.orientation.x, out.pose.pose.orientation.y = qx, qy
            out.pose.pose.orientation.z, out.pose.pose.orientation.w = qz, qw
            out.twist = msg.twist
            self.fused_pub[r].publish(out)
            T_wb = self.o2m[r] @ T_so      # карта mrg_slam = world (init_pose), поэтому T_world_map = 1
            if not self.slam_active[r]:
                d = np.hypot(*(T_wb[:2, 3] - (self.spawn[r] @ T_ob)[:2, 3]))
                self.get_logger().info(f'{r}: SLAM включён; расхождение со стартовой привязкой {d:.2f} м')
                self.slam_active[r] = True
            self.t_wo[r] = world_odom(T_wb, T_ob, self.base_z)
            if self.trace:
                # поза по SLAM и по одним колёсам (от точки старта) на штамп скана
                T_ww = self.spawn[r] @ T_ob
                T_rr = self.o2m[r] @ T_raw   # чистый scan matching, для сравнения
                self.trace.writerow([f'{stamp.nanoseconds * 1e-9:.2f}', r, f'{self.now_s() - stamp.nanoseconds * 1e-9:.2f}',
                                     f'{T_wb[0, 3]:.3f}', f'{T_wb[1, 3]:.3f}', f'{math.degrees(yaw_of(T_wb)):.2f}',
                                     f'{T_ww[0, 3]:.3f}', f'{T_ww[1, 3]:.3f}', f'{math.degrees(yaw_of(T_ww)):.2f}',
                                     f'{T_rr[0, 3]:.3f}', f'{T_rr[1, 3]:.3f}'] + self.step_cols(r))

    def step_cols(self, r):
        st = self.last_step[r]
        if st is None:
            return [''] * 13
        dS, dW, info, xy, yaw, dg = st
        i = info if info is not None else (float('nan'),) * 3
        return [f'{dS[0, 3]:.4f}', f'{dS[1, 3]:.4f}', f'{math.degrees(yaw_of(dS)):.3f}',
                f'{dW[0, 3]:.4f}', f'{dW[1, 3]:.4f}', f'{math.degrees(yaw_of(dW)):.3f}',
                f'{i[0]:.1f}', f'{i[1]:.1f}', f'{i[2]:.1f}', f'{xy[0]:.4f}', f'{xy[1]:.4f}', f'{math.degrees(yaw):.3f}', f'{math.degrees(dg):.3f}']

    def tick(self):
        for r in self.robots:
            self.process(r)
    def publish_tf(self):
        stamp = (self.get_clock().now() + Duration(seconds=float(self.get_parameter('tf_future_s').value))).to_msg()
        msgs = []
        for r in self.robots:
            T = self.t_wo[r]
            m = TransformStamped()
            m.header.stamp = stamp
            m.header.frame_id = 'world'
            m.child_frame_id = f'{r}/odom'
            m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, T[:3, 3])
            qx, qy, qz, qw = mat_to_quat(T[:3, :3])
            m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = qx, qy, qz, qw
            msgs.append(m)
        self.br.sendTransform(msgs)

    def report(self):
        for r in self.robots:
            self.get_logger().info(f'{r}: сканов {self.n_scan[r]}, odom2map {self.n_o2m[r]}, пропущено без колёс. TF {self.n_fail[r]}, '
                f'шагов без оценки вырожденности (колёса) {self.n_clip[r]}, с прыжком scan matching {self.n_jump[r]}, с поворотом от колёс {self.n_yaw_wheel[r]}')


def main():
    rclpy.init()
    node = SlamAdapter()
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

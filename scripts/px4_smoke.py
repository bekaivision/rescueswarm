#!/usr/bin/env python3
"""Дымовая проверка PX4 SITL из ROS 2 (Фаза 5, WP-A). Запускается в контейнере из scripts/px4_smoke.sh.

Режим run: два дрона PX4 (px4_1, px4_2) летят в режиме offboard: ждём валидную локальную позицию,
стримим уставки, включаем offboard, взлёт, квадрат 6 м со скоростью 2 м/с, возврат, посадка.
Все таймеры идут по времени симуляции (use_sim_time, мост /clock), поэтому работает и при RTF < 1.
Пишет в --out: samples_drone_<i>.csv (оценка EKF и истинная поза Gazebo), result.json.

Режим summarize: собирает summary.md из result.json нескольких прогонов.

Системы координат: PX4 NED, мир Gazebo ENU. east = y_ned, north = x_ned, up = -z_ned;
начало NED у каждого дрона своё (точка спауна), поэтому оценку сдвигаем на истинную позу при взлёте.
"""
import argparse
import csv
import json
import math
import os
import statistics
import sys
import time


# ---------------------------------------------------------------- мониторинг процессов (/proc)
def find_pids():
    """Возвращает {имя: [pid,...]} для процессов PX4, агента и сервера gz."""
    res = {}
    for d in os.listdir('/proc'):
        if not d.isdigit():
            continue
        try:
            with open(f'/proc/{d}/cmdline', 'rb') as f:
                args = f.read().decode(errors='replace').split('\0')
        except OSError:
            continue
        cmd = ' '.join(args)
        name = None
        if '/bin/px4' in args[0] or args[0].endswith('/px4'):
            inst = args[args.index('-i') + 1] if '-i' in args else '0'
            name = f'px4_{inst}'
        elif 'MicroXRCEAgent' in args[0]:
            name = 'agent'
        elif 'gz sim' in cmd and ' -s' in cmd and 'ruby' in cmd or 'gz-sim-server' in cmd:
            name = 'gz_server'
        if name:
            res.setdefault(name, []).append(int(d))
    return res


def proc_ticks_rss(pid):
    try:
        with open(f'/proc/{pid}/stat') as f:
            s = f.read()
        p = s[s.rindex(')') + 2:].split()
        ticks = int(p[11]) + int(p[12])
        with open(f'/proc/{pid}/statm') as f:
            rss = int(f.read().split()[1]) * os.sysconf('SC_PAGE_SIZE')
        return ticks, rss
    except (OSError, ValueError, IndexError):
        return None


class ProcMon:
    def __init__(self):
        self.last = {}      # pid -> (wall, ticks)
        self.rows = []      # (wall, name, cpu_percent, rss_mb)
        self.hz = os.sysconf('SC_CLK_TCK')

    def sample(self):
        now = time.time()
        for name, pids in find_pids().items():
            for pid in pids:
                tr = proc_ticks_rss(pid)
                if tr is None:
                    continue
                ticks, rss = tr
                if pid in self.last:
                    w0, t0 = self.last[pid]
                    if now > w0:
                        cpu = 100.0 * (ticks - t0) / self.hz / (now - w0)
                        self.rows.append((now, name, cpu, rss / 1e6))
                self.last[pid] = (now, ticks)

    def stats(self, t_from, t_to):
        out = {}
        for name in sorted({r[1] for r in self.rows}):
            sel = [r for r in self.rows if r[1] == name and t_from <= r[0] <= t_to]
            if sel:
                cpu = [r[2] for r in sel]
                ram = [r[3] for r in sel]
                out[name] = dict(cpu_mean=statistics.mean(cpu), cpu_max=max(cpu),
                                 ram_mean_mb=statistics.mean(ram), ram_max_mb=max(ram), n=len(sel))
        return out


# ---------------------------------------------------------------- статистика ошибок
def err_stats(e):
    e = [float(x) for x in e]
    if not e:
        return dict(mean=float('nan'), rmse=float('nan'), max=float('nan'), n=0)
    a = [abs(x) for x in e]
    return dict(mean=sum(a) / len(a), rmse=math.sqrt(sum(x * x for x in e) / len(e)), max=max(a), n=len(e))


def square_ref(t, side, speed):
    """Опорная точка и скорость (east, north) на квадрате; t - секунды от начала, 4 стороны."""
    edge_t = side / speed
    corners = [(0, 0), (side, 0), (side, side), (0, side), (0, 0)]
    if t >= 4 * edge_t:
        return corners[-1], (0.0, 0.0), True
    k = int(t // edge_t)
    f = (t - k * edge_t) / edge_t
    a, b = corners[k], corners[k + 1]
    p = (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)
    v = ((b[0] - a[0]) / edge_t, (b[1] - a[1]) / edge_t)
    return p, v, False


# ---------------------------------------------------------------- узел
def run(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus,
                              VehicleLocalPosition, VehicleOdometry, VehicleLandDetected)
    # Истинные позы берём прямо из gz-transport: мост Pose_V -> TFMessage теряет имена моделей
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V

    os.makedirs(args.out, exist_ok=True)
    n = args.drones
    spawn_names = [f'{args.model}_{i + 1}' for i in range(n)]
    alts = [args.alt0 + i * args.alt_step for i in range(n)]
    px4_t0 = [float(x) for x in args.px4_t0.split(',')] if args.px4_t0 else [time.time()] * n

    class Smoke(Node):
        def __init__(self):
            super().__init__('px4_smoke', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
            be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                            history=HistoryPolicy.KEEP_LAST, depth=5)
            self.topic_log = []
            self.d = []
            for i in range(n):
                ns = f'/px4_{i + 1}/fmu'
                # экземпляр PX4 = i+1, MAV_SYS_ID = экземпляр + 1
                st = dict(i=i, ns=ns, target=i + 2)
                st['pub_ocm'] = self.create_publisher(OffboardControlMode, f'{ns}/in/offboard_control_mode', 10)
                st['pub_sp'] = self.create_publisher(TrajectorySetpoint, f'{ns}/in/trajectory_setpoint', 10)
                st['pub_cmd'] = self.create_publisher(VehicleCommand, f'{ns}/in/vehicle_command', 10)
                v = lambda msg: f'_v{msg.MESSAGE_VERSION}' if getattr(msg, 'MESSAGE_VERSION', 0) else ''
                st['topics'] = dict(
                    status=f'{ns}/out/vehicle_status{v(VehicleStatus)}',
                    lpos=f'{ns}/out/vehicle_local_position{v(VehicleLocalPosition)}',
                    odom=f'{ns}/out/vehicle_odometry{v(VehicleOdometry)}',
                    land=f'{ns}/out/vehicle_land_detected{v(VehicleLandDetected)}')
                self.create_subscription(VehicleStatus, st['topics']['status'], lambda m, s=st: self.on_status(s, m), be)
                self.create_subscription(VehicleLocalPosition, st['topics']['lpos'], lambda m, s=st: self.on_lpos(s, m), be)
                self.create_subscription(VehicleOdometry, st['topics']['odom'], lambda m, s=st: self.on_odom(s, m), be)
                self.create_subscription(VehicleLandDetected, st['topics']['land'], lambda m, s=st: self.on_land(s, m), be)
                st.update(armed=False, nav=-1, checks=False, xy=False, z=False, landed=True, ready_wall=None,
                          lp=None, rows=[], gt=None, spawn=None, got_status=False, got_odom=False, got_lpos=False,
                          phase='WAIT', t_phase=None, sq_t0=None, arm_t=None, cmd_t=-10.0, land_t=None,
                          reached_alt=False, done=False, ok=True, notes=[], t_arm_wall=None, t_land_wall=None,
                          sq_done=False, ts_lag=[])
                self.d.append(st)
            self.gz = GzNode()
            if not self.gz.subscribe(Pose_V, args.pose_topic, self.on_pose):
                self.get_logger().error(f'не удалось подписаться на {args.pose_topic} (gz)')
            self.pm = ProcMon()
            self.create_timer(0.02, self.tick)
            self.pm_timer = self.create_timer(2.0, self.pm.sample)
            self.wall0 = time.time()
            self.sim0 = None
            self.clk_log = []        # (wall, sim)
            self.t_start_wall = time.time()
            self.t_end_wall = None
            self.logged_topics = False
            self.printed_fields = False

        # ---- вспомогательное
        def now_us(self):
            return self.get_clock().now().nanoseconds // 1000

        def sim_s(self):
            return self.get_clock().now().nanoseconds / 1e9

        def cmd(self, s, command, p1=0.0, p2=0.0):
            m = VehicleCommand()
            m.timestamp = self.now_us()
            m.command = command
            m.param1 = float(p1)
            m.param2 = float(p2)
            m.target_system = s['target']
            m.target_component = 1
            m.source_system = 1
            m.source_component = 1
            m.from_external = True
            s['pub_cmd'].publish(m)

        # ---- подписки
        def on_pose(self, msg):
            # поток gz-transport: только присваивание кортежа (атомарно для GIL)
            for p in msg.pose:
                for s, nm in zip(self.d, spawn_names):
                    if p.name == nm:
                        s['gt'] = (p.position.x, p.position.y, p.position.z)

        def on_status(self, s, m):
            s['got_status'] = True
            s['armed'] = (m.arming_state == getattr(VehicleStatus, 'ARMING_STATE_ARMED', 2))
            s['nav'] = m.nav_state
            s['checks'] = bool(getattr(m, 'pre_flight_checks_pass', True))
            if not self.printed_fields:
                self.printed_fields = True
                self.get_logger().info('VehicleStatus fields: ' + ', '.join(m.get_fields_and_field_types().keys()))

        def on_lpos(self, s, m):
            s['got_lpos'] = True
            s['xy'] = bool(m.xy_valid)
            s['z'] = bool(m.z_valid)
            s['lp'] = (m.x, m.y, m.z, m.vx, m.vy, m.vz)
            s['ts_lag'].append(self.now_us() - m.timestamp)

        def on_land(self, s, m):
            s['landed'] = bool(m.landed)

        def on_odom(self, s, m):
            s['got_odom'] = True
            if s['gt'] is None or s['spawn'] is None:
                return
            e, nn, u = m.position[1], m.position[0], -m.position[2]
            g = s['gt']
            sp = s['spawn']
            s['rows'].append((self.sim_s(), time.time(), s['phase'], e + sp[0], nn + sp[1], u + sp[2], g[0], g[1], g[2]))

        # ---- основной цикл (по времени симуляции)
        def tick(self):
            t = self.sim_s()
            if t <= 0.0:
                return
            if self.sim0 is None:
                self.sim0 = t
            self.clk_log.append((time.time(), t))
            if not self.logged_topics and time.time() - self.wall0 > 5.0:
                self.logged_topics = True
                names = sorted(n_ for n_, _ in self.get_topic_names_and_types() if '/fmu/' in n_)
                with open(os.path.join(args.out, 'topics.txt'), 'w') as f:
                    f.write('\n'.join(names) + '\n')
            if time.time() - self.t_start_wall > args.wall_timeout:
                self.get_logger().error('wall timeout')
                for s in self.d:
                    s['ok'] = False
                    s['notes'].append('wall timeout in phase ' + s['phase'])
                self.finish()
                return
            for s in self.d:
                if s['spawn'] is None and s['gt'] is not None:
                    s['spawn'] = s['gt']
                self.step(s, t)
            if all(s['done'] for s in self.d):
                self.finish()

        def publish_sp(self, s, pos_enu, vel_enu=None, yaw_ned=math.pi / 2):
            """pos_enu: (east, north, up) относительно точки спауна; в PX4 NED."""
            ocm = OffboardControlMode()
            ocm.timestamp = self.now_us()
            ocm.position = True
            ocm.velocity = False
            ocm.acceleration = False
            ocm.attitude = False
            ocm.body_rate = False
            s['pub_ocm'].publish(ocm)
            sp = TrajectorySetpoint()
            sp.timestamp = self.now_us()
            sp.position = [float(pos_enu[1]), float(pos_enu[0]), float(-pos_enu[2])]
            nan = float('nan')
            sp.velocity = [float(vel_enu[1]), float(vel_enu[0]), nan] if vel_enu else [nan, nan, nan]
            sp.acceleration = [nan, nan, nan]
            sp.jerk = [nan, nan, nan]
            sp.yaw = float(yaw_ned)
            sp.yawspeed = nan
            s['pub_sp'].publish(sp)

        def set_phase(self, s, ph, t):
            self.get_logger().info(f'drone_{s["i"] + 1}: {s["phase"]} -> {ph} (sim {t:.1f}s)')
            s['phase'] = ph
            s['t_phase'] = t

        def step(self, s, t):
            ph = s['phase']
            alt = alts[s['i']]
            i = s['i']
            if ph == 'WAIT':
                if s['got_status'] and s['got_lpos'] and s['xy'] and s['z'] and s['checks'] and s['spawn'] is not None:
                    if s['ready_wall'] is None:
                        s['ready_wall'] = time.time()
                        self.get_logger().info(f'drone_{i + 1} ready to arm {s["ready_wall"] - px4_t0[i]:.1f} s after PX4 start')
                    # ждём готовности обоих дронов, потом начинаем стрим
                    if all(x['ready_wall'] is not None for x in self.d):
                        self.set_phase(s, 'STREAM', t)
                return
            # дальше уставки идут всегда (минимум 2 Гц нужно PX4 в сим. времени)
            if ph == 'STREAM':
                self.publish_sp(s, (0.0, 0.0, 0.0))
                if t - s['t_phase'] > 1.5:
                    self.set_phase(s, 'ARMING', t)
                    s['arm_t'] = t
                return
            if ph == 'ARMING':
                self.publish_sp(s, (0.0, 0.0, alt))
                if t - s['cmd_t'] >= 1.0 and not (s['armed'] and s['nav'] == 14):
                    s['cmd_t'] = t
                    self.cmd(s, 176, 1.0, 6.0)      # DO_SET_MODE, custom mode, offboard
                    self.cmd(s, 400, 1.0)           # ARM
                if s['armed'] and s['nav'] == 14:
                    s['t_arm_wall'] = time.time()
                    # истинная поза на земле перед взлётом = сдвиг NED -> мир
                    s['spawn'] = s['gt']
                    self.set_phase(s, 'CLIMB', t)
                elif t - s['arm_t'] > 60.0:
                    s['ok'] = False
                    s['notes'].append('не удалось включить offboard/arm за 60 с')
                    s['done'] = True
                return
            if ph == 'CLIMB':
                self.publish_sp(s, (0.0, 0.0, alt))
                z_up = -s['lp'][2] if s['lp'] else 0.0
                if abs(z_up - alt) < 0.3 and abs(s['lp'][5]) < 0.3:
                    s['reached_alt'] = True
                    self.set_phase(s, 'HOVER', t)
                elif t - s['t_phase'] > 40.0:
                    s['ok'] = False
                    s['notes'].append(f'взлёт: высота {z_up:.2f} м вместо {alt} м за 40 с')
                    self.set_phase(s, 'LAND', t)
                    self.cmd(s, 21)
                return
            if ph == 'HOVER':
                self.publish_sp(s, (0.0, 0.0, alt))
                if t - s['t_phase'] > 3.0 and all(x['phase'] in ('HOVER', 'SQUARE', 'RETURN', 'LAND', 'DONE') or x['done'] for x in self.d):
                    s['sq_t0'] = t
                    self.set_phase(s, 'SQUARE', t)
                return
            if ph == 'SQUARE':
                p, v, fin = square_ref(t - s['sq_t0'], args.side, args.speed)
                self.publish_sp(s, (p[0], p[1], alt), (v[0], v[1], 0.0) if not fin else None)
                if fin:
                    s['sq_done'] = True
                    self.set_phase(s, 'RETURN', t)
                return
            if ph == 'RETURN':      # висим над точкой старта, ждём пока подойдёт и успокоится
                self.publish_sp(s, (0.0, 0.0, alt))
                lp = s['lp']
                near = lp and math.hypot(lp[0], lp[1]) < 0.4 and math.hypot(lp[3], lp[4]) < 0.3
                if near or t - s['t_phase'] > 15.0:
                    self.set_phase(s, 'LAND', t)
                    s['land_t'] = t
                    self.cmd(s, 21)         # NAV_LAND
                    s['cmd_t'] = t
                return
            if ph == 'LAND':
                if s['land_t'] is None:
                    s['land_t'] = t
                if t - s['cmd_t'] >= 2.0 and s['armed']:
                    s['cmd_t'] = t
                    self.cmd(s, 21)
                if (s['landed'] and not s['armed']) or (s['landed'] and t - s['land_t'] > 20.0):
                    s['t_land_wall'] = time.time()
                    self.set_phase(s, 'DONE', t)
                    s['done'] = True
                elif t - s['land_t'] > 90.0:
                    s['ok'] = False
                    s['notes'].append('посадка: не приземлился за 90 с')
                    s['done'] = True
                return

        # ---- итог
        def finish(self):
            if self.t_end_wall is not None:
                return
            self.t_end_wall = time.time()
            self.pm.sample()
            res = dict(world=args.world_label, drones=[], wall_s=self.t_end_wall - self.t_start_wall)
            clk = self.clk_log
            # RTF по всему прогону и по полёту (от первой армирования до последней посадки)
            def rtf(a, b):
                sel = [c for c in clk if a <= c[0] <= b]
                if len(sel) < 2 or sel[-1][0] <= sel[0][0]:
                    return float('nan')
                return (sel[-1][1] - sel[0][1]) / (sel[-1][0] - sel[0][0])
            arms = [s['t_arm_wall'] for s in self.d if s['t_arm_wall']]
            lands = [s['t_land_wall'] or self.t_end_wall for s in self.d]
            res['rtf_total'] = rtf(self.t_start_wall, self.t_end_wall)
            res['rtf_flight'] = rtf(min(arms), max(lands)) if arms else float('nan')
            f_from = min(arms) if arms else self.t_start_wall
            f_to = max(lands)
            res['proc_flight'] = self.pm.stats(f_from, f_to)
            res['proc_all'] = self.pm.stats(self.t_start_wall, self.t_end_wall)
            for s in self.d:
                i = s['i']
                rows = s['rows']
                with open(os.path.join(args.out, f'samples_drone_{i + 1}.csv'), 'w', newline='') as f:
                    w = csv.writer(f)
                    w.writerow(['sim_t', 'wall_t', 'phase', 'est_e', 'est_n', 'est_u', 'gt_e', 'gt_n', 'gt_u'])
                    w.writerows(rows)
                fl = [r for r in rows if r[2] in ('CLIMB', 'HOVER', 'SQUARE', 'RETURN', 'LAND')]
                ex = [r[3] - r[6] for r in fl]
                ey = [r[4] - r[7] for r in fl]
                ez = [r[5] - r[8] for r in fl]
                exy = [math.hypot(a, b) for a, b in zip(ex, ey)]
                sq = [r for r in rows if r[2] in ('HOVER', 'SQUARE', 'RETURN')]
                sp = s['spawn'] or (0, 0, 0)
                alt = alts[i]
                hold_gt = [r[8] - sp[2] - alt for r in sq]       # истинная высота над точкой взлёта минус заданная
                hold_est = [r[5] - sp[2] - alt for r in sq]
                lag = s['ts_lag']
                res['drones'].append(dict(
                    drone=i + 1, ok=s['ok'], notes=s['notes'], target_alt=alt,
                    ready_s=(s['ready_wall'] - px4_t0[i]) if s['ready_wall'] else None,
                    reached_alt=s['reached_alt'], square_done=s['sq_done'], landed=bool(s['t_land_wall']),
                    err_xy=err_stats(exy), err_x=err_stats(ex), err_y=err_stats(ey), err_z=err_stats(ez),
                    hold_gt=err_stats(hold_gt), hold_est=err_stats(hold_est),
                    n_samples=len(rows), spawn=list(sp),
                    ts_lag_ms_mean=(statistics.mean(lag) / 1000.0 if lag else None),
                    ts_lag_ms_min=(min(lag) / 1000.0 if lag else None),
                    ts_lag_ms_max=(max(lag) / 1000.0 if lag else None),
                    topics=s['topics']))
            res['ok'] = all(d_['ok'] and d_['reached_alt'] and d_['square_done'] and d_['landed'] for d_ in res['drones'])
            for d_ in res['drones']:
                if d_['err_xy']['rmse'] > args.max_rmse_xy or d_['err_z']['rmse'] > args.max_rmse_z or d_['err_xy']['n'] < 50:
                    res['ok'] = False
                    d_['notes'].append('ошибка EKF выше порога или мало отсчётов')
            with open(os.path.join(args.out, 'result.json'), 'w') as f:
                json.dump(res, f, indent=1)
            self.result = res
            self.get_logger().info('результат: ' + ('OK' if res['ok'] else 'FAIL'))
            raise SystemExit(0 if res['ok'] else 1)

    rclpy.init()
    node = Smoke()
    try:
        rclpy.spin(node)
    except SystemExit as e:
        code = e.code
    else:
        code = 2
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


# ---------------------------------------------------------------- сводка
def fmt(x, nd=2):
    return 'n/a' if x is None or (isinstance(x, float) and math.isnan(x)) else f'{x:.{nd}f}'


def summarize(args):
    lines = ['# PX4 smoke: сводка', '']
    for d in args.dirs:
        p = os.path.join(d, 'result.json')
        if not os.path.exists(p):
            lines += [f'## {os.path.basename(d.rstrip("/"))}', 'result.json нет (прогон не дошёл до конца)', '']
            continue
        r = json.load(open(p))
        lines += [f'## Мир: {r["world"]}  ({"OK" if r["ok"] else "FAIL"})', '',
                  f'Длительность прогона {fmt(r["wall_s"], 0)} с (реальное время). RTF: весь прогон {fmt(r["rtf_total"])}, полёт {fmt(r["rtf_flight"])}.', '',
                  '| дрон | готов к arm, с | взлёт | квадрат | посадка | xy mean/rmse/max, м | z mean/rmse/max, м | удержание высоты (истина) mean/max, м | отсчётов |',
                  '|---|---|---|---|---|---|---|---|---|']
        for x in r['drones']:
            lines.append('| {} | {} | {} | {} | {} | {}/{}/{} | {}/{}/{} | {}/{} | {} |'.format(
                x['drone'], fmt(x['ready_s'], 1), 'да' if x['reached_alt'] else 'НЕТ', 'да' if x['square_done'] else 'НЕТ',
                'да' if x['landed'] else 'НЕТ', fmt(x['err_xy']['mean']), fmt(x['err_xy']['rmse']), fmt(x['err_xy']['max']),
                fmt(x['err_z']['mean']), fmt(x['err_z']['rmse']), fmt(x['err_z']['max']),
                fmt(x['hold_gt']['mean']), fmt(x['hold_gt']['max']), x['n_samples']))
            for nt in x['notes']:
                lines.append(f'|  | примечание: {nt} | | | | | | | |')
        lines += ['', 'Процессы за время полёта (CPU в процентах одного ядра, RAM в МБ; среднее/максимум):', '',
                  '| процесс | CPU mean | CPU max | RAM mean | RAM max |', '|---|---|---|---|---|']
        for name, v in r['proc_flight'].items():
            lines.append(f'| {name} | {fmt(v["cpu_mean"], 0)} | {fmt(v["cpu_max"], 0)} | {fmt(v["ram_mean_mb"], 0)} | {fmt(v["ram_max_mb"], 0)} |')
        lines.append('')
    with open(args.output, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='mode', required=True)
    r = sub.add_parser('run')
    r.add_argument('--out', required=True)
    r.add_argument('--world', default='default', help='имя мира в SDF (атрибут name)')
    r.add_argument('--world-label', default='default')
    r.add_argument('--model', default='x500', help='префикс имени модели: <model>_<instance>')
    r.add_argument('--drones', type=int, default=2)
    r.add_argument('--alt0', type=float, default=4.5)
    r.add_argument('--alt-step', type=float, default=1.0)
    r.add_argument('--side', type=float, default=6.0)
    r.add_argument('--speed', type=float, default=2.0)
    r.add_argument('--px4-t0', default='', help='реальное время старта PX4 каждого дрона, через запятую (epoch)')
    r.add_argument('--wall-timeout', type=float, default=600.0)
    r.add_argument('--max-rmse-xy', type=float, default=0.5)
    r.add_argument('--max-rmse-z', type=float, default=0.5)
    s = sub.add_parser('summarize')
    s.add_argument('--output', required=True)
    s.add_argument('dirs', nargs='+')
    a = ap.parse_args()
    if a.mode == 'run':
        a.pose_topic = f'/world/{a.world}/pose/info'
        run(a)
    else:
        summarize(a)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Вспомогательные узлы для slam_smoke.launch.py (Фаза 4, проверка mrg_slam на одном роботе).

Режимы (первый аргумент):
  relay <робот>   - читает /<робот>/lidar/points, меняет frame_id на <робот>/slam_lidar,
                    публикует в /<робот>/velodyne_points (вход mrg_slam). Зачем: см. config/mrg_slam_ground.yaml.
  run <робот> <длительность_сим_с> <out_dir>
                  - ведёт робота по кругу (cmd_vel), раз в 2 с (сим.) сравнивает позу SLAM
                    (TF world -> <робот>/slam_base) с истинной (/<робот>/gt_odom), раз в 10 с (сим.)
                    меряет CPU/RAM процессов и RTF; в конце пишет pose_error.csv, resources.csv, summary.json.
"""
import csv
import json
import math
import os
import sys
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

SIM = [rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)]


class Relay(Node):
    def __init__(self, robot):
        super().__init__('slam_cloud_relay', parameter_overrides=SIM)
        self.frame = f'{robot}/slam_lidar'
        self.pub = self.create_publisher(PointCloud2, f'/{robot}/velodyne_points', 10)
        self.create_subscription(PointCloud2, f'/{robot}/lidar/points', self.cb, 10)

    def cb(self, msg):
        msg.header.frame_id = self.frame
        self.pub.publish(msg)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def proc_table():
    """pid -> (имя, тики CPU, RSS в КБ) из /proc."""
    out = {}
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/stat') as f:
                st = f.read()
            with open(f'/proc/{pid}/cmdline') as f:
                cmd = f.read().replace('\0', ' ')
            rest = st[st.rindex(')') + 2:].split()
            ticks = int(rest[11]) + int(rest[12])
            rss = int(rest[21]) * (os.sysconf('SC_PAGE_SIZE') // 1024)
            out[int(pid)] = (cmd, ticks, rss)
        except (OSError, ValueError, IndexError):
            continue
    return out


def group_of(cmd):
    if 'mrg_slam_container' in cmd or 'component_container_mt' in cmd:
        return 'mrg_slam_container'
    if 'map2odom_publisher' in cmd:
        return 'map2odom'
    if 'gz sim' in cmd and '-s' in cmd:
        return 'gz_server'
    if 'gz sim' in cmd:
        return 'gz_other'
    if 'parameter_bridge' in cmd:
        return 'bridge'
    if 'slam_smoke_tools.py relay' in cmd:
        return 'relay'
    return None


class Runner(Node):
    def __init__(self, robot, duration, out_dir):
        super().__init__('slam_smoke_runner', parameter_overrides=SIM)
        self.robot, self.duration, self.out_dir = robot, duration, out_dir
        os.makedirs(out_dir, exist_ok=True)
        self.cmd_pub = self.create_publisher(Twist, f'/{robot}/cmd_vel', 10)
        self.gt = deque(maxlen=400)  # (t, x, y, z, yaw)
        self.create_subscription(Odometry, f'/{robot}/gt_odom', self.gt_cb, 20)
        self.cloud_n = 0
        self.map_pts = []  # (t, число точек)
        self.create_subscription(PointCloud2, f'/{robot}/mrg_slam/map_points', self.map_cb,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.tfb = Buffer(cache_time=Duration(seconds=30))
        TransformListener(self.tfb, self)
        self.t0 = None  # старт движения, сим. время
        self.wall0 = None
        self.rows = []
        self.res_rows = []
        self.prev_proc = None
        self.prev_clock = None
        self.slam_first_t = None
        self.create_timer(0.05, self.drive)      # 20 Гц
        self.create_timer(2.0, self.compare)     # сим. время
        self.create_timer(10.0, self.resources)  # сим. время
        self.done = False
        self.wall_start = time.time()
        # сторожок по настенным часам: если нет /clock или SLAM не стартовал, не висим вечно
        self.create_timer(5.0, self.watchdog, clock=Clock(clock_type=ClockType.SYSTEM_TIME))

    def watchdog(self):
        waited = time.time() - self.wall_start
        if self.t0 is None and waited > 120:
            self.get_logger().error('SLAM не выдал позу за 120 с (настенных): нет /clock или mrg_slam не стартовал')
            raise SystemExit(1)
        if self.t0 is not None and time.time() - self.wall0 > 3 * self.duration + 120:
            self.get_logger().error('прогон идёт слишком долго по настенным часам, RTF слишком мал')
            self.finish()

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def gt_cb(self, m):
        p = m.pose.pose
        self.gt.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                        p.position.x, p.position.y, p.position.z, yaw_of(p.orientation)))

    def map_cb(self, m):
        self.map_pts.append((self.now_s(), m.width * m.height))

    # --- движение: круг радиуса v/w, 0.5 м/с, 0.2 рад/с -> радиус 2.5 м, виток 31.4 с
    def drive(self):
        if self.done or not self.gt:
            return
        t = self.now_s()
        if self.t0 is None:
            if self.slam_first_t is None:
                return  # ждём, пока SLAM начнёт публиковать позу
            self.t0, self.wall0 = t, time.time()
            self.get_logger().info('старт движения')
        el = t - self.t0
        msg = Twist()
        if el < self.duration:
            msg.linear.x, msg.angular.z = 0.5, 0.2
        self.cmd_pub.publish(msg)
        if el >= self.duration + 3.0:
            self.finish()

    def gt_at(self, t):
        g = list(self.gt)
        for a, b in zip(g, g[1:]):
            if a[0] <= t <= b[0]:
                k = (t - a[0]) / max(b[0] - a[0], 1e-9)
                dy = math.atan2(math.sin(b[4] - a[4]), math.cos(b[4] - a[4]))
                return (a[1] + k * (b[1] - a[1]), a[2] + k * (b[2] - a[2]),
                        a[3] + k * (b[3] - a[3]), a[4] + k * dy)
        return None

    def compare(self):
        try:
            tf = self.tfb.lookup_transform(f'{self.robot}/slam_odom', f'{self.robot}/slam_base', Time())
            s = Time.from_msg(tf.header.stamp)
            w = self.tfb.lookup_transform('world', f'{self.robot}/slam_base', s)
        except Exception:
            return
        if self.slam_first_t is None:
            self.slam_first_t = self.now_s()
            self.get_logger().info('получена первая поза SLAM')
        if self.t0 is None:
            return
        g = self.gt_at(s.nanoseconds * 1e-9)
        if g is None:
            return
        p = w.transform.translation
        yaw = yaw_of(w.transform.rotation)
        ex, ey = p.x - g[0], p.y - g[1]
        eyaw = math.atan2(math.sin(yaw - g[3]), math.cos(yaw - g[3]))
        self.rows.append((self.now_s() - self.t0, g[0], g[1], g[3], p.x, p.y, yaw,
                          math.hypot(ex, ey), math.degrees(eyaw)))

    def resources(self):
        wall, sim = time.time(), self.now_s()
        cur = proc_table()
        if self.prev_proc is not None:
            dw = wall - self.prev_proc[0]
            rtf = (sim - self.prev_clock) / dw if dw > 0 else 0.0
            tick = os.sysconf('SC_CLK_TCK')
            cpu, rss = {}, {}
            for pid, (cmd, ticks, kb) in cur.items():
                g = group_of(cmd)
                if g is None:
                    continue
                old = self.prev_proc[1].get(pid)
                if old:
                    cpu[g] = cpu.get(g, 0.0) + 100.0 * (ticks - old[1]) / tick / dw
                rss[g] = rss.get(g, 0) + kb / 1024.0
            self.res_rows.append((sim - (self.t0 or sim), rtf, cpu, rss))
        self.prev_proc, self.prev_clock = (wall, cur), sim

    def finish(self):
        self.done = True
        self.cmd_pub.publish(Twist())
        r = self.rows
        with open(os.path.join(self.out_dir, 'pose_error.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_sim', 'gt_x', 'gt_y', 'gt_yaw', 'slam_x', 'slam_y', 'slam_yaw', 'err_xy_m', 'err_yaw_deg'])
            w.writerows([[f'{v:.4f}' for v in row] for row in r])
        groups = sorted({g for _, _, c, m in self.res_rows for g in list(c) + list(m)})
        with open(os.path.join(self.out_dir, 'resources.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_sim', 'rtf'] + [f'cpu_{g}_pct' for g in groups] + [f'rss_{g}_mb' for g in groups])
            for t, rtf, c, m in self.res_rows:
                w.writerow([f'{t:.1f}', f'{rtf:.3f}'] + [f'{c.get(g, 0):.1f}' for g in groups]
                           + [f'{m.get(g, 0):.0f}' for g in groups])
        s = {'robot': self.robot, 'duration_sim_s': self.duration, 'samples': len(r),
             'map_cloud_msgs': len(self.map_pts),
             'map_cloud_last_points': self.map_pts[-1][1] if self.map_pts else 0}
        if r:
            e = [x[7] for x in r]
            path = 0.0
            for a, b in zip(r, r[1:]):
                path += math.hypot(b[1] - a[1], b[2] - a[2])
            s.update(path_len_m=round(path, 1), err_final_m=round(e[-1], 3), err_final_yaw_deg=round(r[-1][8], 2),
                     err_max_m=round(max(e), 3), err_rmse_m=round(math.sqrt(sum(v * v for v in e) / len(e)), 3),
                     drift_pct_of_path=round(100 * e[-1] / path, 2) if path > 0 else None)
        rr = [x for x in self.res_rows if x[0] > 5]
        if rr:
            def avg(sel):
                v = [sel(x) for x in rr]
                return sum(v) / len(v)
            s['rtf_mean'] = round(avg(lambda x: x[1]), 3)
            for g in groups:
                s[f'cpu_{g}_pct_mean'] = round(avg(lambda x: x[2].get(g, 0)), 1)
                s[f'cpu_{g}_pct_max'] = round(max(x[2].get(g, 0) for x in rr), 1)
                s[f'rss_{g}_mb_last'] = round(rr[-1][3].get(g, 0))
        with open(os.path.join(self.out_dir, 'summary.json'), 'w') as f:
            json.dump(s, f, indent=2, ensure_ascii=False)
        self.get_logger().info('SUMMARY ' + json.dumps(s, ensure_ascii=False))
        raise SystemExit(0)


def main():
    rclpy.init()
    mode = sys.argv[1]
    node = Relay(sys.argv[2]) if mode == 'relay' else Runner(sys.argv[2], float(sys.argv[3]), sys.argv[4])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

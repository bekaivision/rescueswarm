#!/usr/bin/env python3
"""Сценарий движения для видео Фазы 1 (запускается внутри контейнера).
Каждый робот едет по своей программе, время берётся из /clock (симуляция),
поэтому движение не зависит от скорости ноутбука.
  phase1_drive.py [секунд_симуляции]
Программа: список (длительность, v, w).
"""
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from nav_msgs.msg import Odometry

PROGRAM = {
    # ground_1: прямо к нагретому телу, замедляется и осторожно доворачивает налево (к цели не касается)
    'ground_1': [(4.0, 0.40, 0.0), (4.0, 0.25, 0.0), (99.0, 0.10, 0.15)],
    # ground_2: едет, крутится на месте, снова едет
    'ground_2': [(3.0, 0.35, 0.0), (5.0, 0.0, 1.2), (99.0, 0.35, 0.0)],
    # ground_3: дуга налево, потом дуга направо (змейка)
    'ground_3': [(5.0, 0.35, 0.25), (99.0, 0.35, -0.30)],
}


class Drive(Node):
    def __init__(self, total):
        super().__init__('phase1_drive', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.total = total
        self.t0 = None
        self.pubs = {ns: self.create_publisher(Twist, f'/{ns}/cmd_vel', 10) for ns in PROGRAM}
        self.odom = {}
        for ns in PROGRAM:
            self.create_subscription(Odometry, f'/{ns}/odom',
                                     lambda m, ns=ns: self.odom.__setitem__(ns, m.pose.pose.position), 10)
        self.create_timer(0.05, self.tick)
        self.done = False
        self.last_log = -1

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        for ns, prog in PROGRAM.items():
            e = 0.0
            cmd = Twist()
            for dur, v, w in prog:
                e += dur
                if t < e:
                    cmd.linear.x, cmd.angular.z = v, w
                    break
            self.pubs[ns].publish(cmd)
        if int(t) != self.last_log and int(t) % 2 == 0:
            self.last_log = int(t)
            pos = ' '.join(f'{ns}=({p.x:.1f},{p.y:.1f})' for ns, p in sorted(self.odom.items()))
            print(f't={t:4.1f} {pos}', flush=True)
        if t >= self.total:
            for p in self.pubs.values():
                p.publish(Twist())
            self.done = True


def main():
    rclpy.init()
    node = Drive(float(sys.argv[1]) if len(sys.argv) > 1 else 14.0)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

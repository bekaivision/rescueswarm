"""Узел other_robots: роботы видят друг друга по истинным позам.

Для каждого робота i публикует /<robot_i>/other_robots_scan (sensor_msgs/LaserScan, кадр
<robot_i>/base_link): виртуальный 360-градусный скан, в котором есть только корпуса остальных
роботов (остальные лучи inf). Его используют:
  - costmap Nav2 (слой robots_layer: отметка + стирание),
  - nav2_collision_monitor (аварийная остановка независимо от планировщика).
Ещё публикует /fleet/poses (geometry_msgs/PoseArray, кадр world, порядок = параметр robots).
В Фазе 6 те же позы придут через comms_sim: роботы и так рассылают свои координаты.
Параметры: robots, rate (Гц), n_rays, half_len, half_wid, pad, max_range, poses_rate.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from rs_mapping.footprint_scan import make_angles, ray_rect_ranges
from rcl_interfaces.msg import ParameterDescriptor


class OtherRobots(Node):
    def __init__(self):
        super().__init__('other_robots')
        P = self.declare_parameter
        P('robots', [], ParameterDescriptor(dynamic_typing=True))
        P('rate', 10.0)
        P('n_rays', 720)
        P('half_len', 0.25)
        P('half_wid', 0.15)
        P('pad', 0.03)
        P('max_range', 8.0)
        P('poses_rate', 5.0)
        self.robots = list(self.get_parameter('robots').value or [])
        if not self.robots:
            raise RuntimeError('параметр robots пуст: список роботов задаёт launch (explore.launch.py / slam.launch.py), запасного списка нет')
        self.n_rays = int(self.get_parameter('n_rays').value)
        self.angles = make_angles(self.n_rays)
        self.max_range = float(self.get_parameter('max_range').value)
        self.poses = {}     # robot -> (x, y, yaw)
        self.stamps = {}    # robot -> stamp последней одометрии
        for r in self.robots:
            self.create_subscription(Odometry, f'/{r}/pose', lambda m, r=r: self._on_odom(r, m),
                                     qos_profile_sensor_data)
        self.pubs = {r: self.create_publisher(LaserScan, f'/{r}/other_robots_scan', 5) for r in self.robots}
        self.poses_pub = self.create_publisher(PoseArray, '/fleet/poses', 5)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._tick)
        self.create_timer(1.0 / float(self.get_parameter('poses_rate').value), self._publish_poses)
        self.get_logger().info(f'other_robots: {self.robots}, {self.n_rays} лучей')

    def _on_odom(self, r, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.poses[r] = (p.x, p.y, yaw)
        self.stamps[r] = msg.header.stamp

    def _tick(self):
        gp = self.get_parameter
        for r in self.robots:
            if r not in self.poses:
                continue
            others = [v for k, v in self.poses.items() if k != r]
            rng = ray_rect_ranges(self.angles, self.poses[r], others, float(gp('half_len').value),
                                  float(gp('half_wid').value), float(gp('pad').value), self.max_range)
            m = LaserScan()
            m.header.stamp = self.stamps[r]     # тот же штамп, что у TF этого робота
            m.header.frame_id = f'{r}/base_link'
            m.angle_min = float(self.angles[0])
            m.angle_max = float(self.angles[-1])
            m.angle_increment = 2.0 * math.pi / self.n_rays
            m.range_min = 0.05
            m.range_max = self.max_range
            m.ranges = rng.astype(np.float32).tolist()
            self.pubs[r].publish(m)

    def _publish_poses(self):
        arr = PoseArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = 'world'
        for r in self.robots:
            p = Pose()
            if r in self.poses:
                x, y, yaw = self.poses[r]
                p.position.x, p.position.y = x, y
                p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            else:
                p.position.x = p.position.y = float('nan')   # поза ещё неизвестна
                p.orientation.w = 1.0
            arr.poses.append(p)
        self.poses_pub.publish(arr)


def main():
    rclpy.init()
    node = OtherRobots()
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

"""Узел pose_from_tf: единая мировая поза роботов /<robot>/pose.

Для каждого робота берёт самый свежий TF world <- <robot>/base_link и публикует его как
nav_msgs/Odometry в кадре world (child_frame_id = <robot>/base_link). Скорости (twist) копируются
из /<robot>/odom, если оно приходит. Все потребители (explorer, shared_map, other_robots, coverage)
читают позу только отсюда: в режиме gt это истинная поза через TF, в режиме slam - поза от SLAM.
Параметры: robots, rate (Гц, 20), global_frame (world).
"""
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class PoseFromTf(Node):
    def __init__(self):
        super().__init__('pose_from_tf')
        self.declare_parameter('robots', ['ground_1', 'ground_2', 'ground_3'])
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('global_frame', 'world')
        self.robots = list(self.get_parameter('robots').value)
        self.frame = self.get_parameter('global_frame').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.twist = {}
        self.last_stamp = {}
        self.pubs = {}
        for r in self.robots:
            self.pubs[r] = self.create_publisher(Odometry, f'/{r}/pose', 10)
            self.create_subscription(Odometry, f'/{r}/odom', lambda m, r=r: self.twist.__setitem__(r, m.twist), 10)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._tick)
        self.get_logger().info(f'pose_from_tf: {self.frame} <- <robot>/base_link, роботы: {self.robots}')

    def _tick(self):
        for r in self.robots:
            try:
                tf = self.tf_buffer.lookup_transform(self.frame, f'{r}/base_link', Time())
            except TransformException:
                continue
            key = (tf.header.stamp.sec, tf.header.stamp.nanosec)
            if self.last_stamp.get(r) == key:
                continue    # TF не обновился, не дублируем
            self.last_stamp[r] = key
            m = Odometry()
            m.header.stamp = tf.header.stamp
            m.header.frame_id = self.frame
            m.child_frame_id = f'{r}/base_link'
            t, q = tf.transform.translation, tf.transform.rotation
            m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = t.x, t.y, t.z
            m.pose.pose.orientation = q
            if r in self.twist:
                m.twist = self.twist[r]
            self.pubs[r].publish(m)


def main():
    rclpy.init()
    node = PoseFromTf()
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

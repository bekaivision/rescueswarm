"""Ретранслятор TF для воспроизведения bag (scripts/slam_replay.sh): из /tf_bag пропускает в /tf только
колёсную одометрию <robot>/odom -> <robot>/base_link. TF world -> odom из bag не нужен: его публикует slam_adapter."""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_msgs.msg import TFMessage

rclpy.init()
n = Node('tfrelay', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
pub = n.create_publisher(TFMessage, '/tf', 100)


def cb(m):
    o = TFMessage()
    o.transforms = [t for t in m.transforms if t.header.frame_id.endswith('/odom') and t.child_frame_id.endswith('/base_link')]
    if o.transforms:
        pub.publish(o)


n.create_subscription(TFMessage, '/tf_bag', cb, 100)
rclpy.spin(n)

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler

class DynamicTFBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        # 动态TF需要持续发布，发布频率设置为 100 Hz
        self.timer = self.create_timer(0.01, self.publish_dynamic_tf)

    def publish_dynamic_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'camera_link'
        transform.child_frame_id = 'bottle_link'
        # 设置平移和旋转
        transform.transform.translation.x = 0.2
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.5
        # 欧拉角转换为四元数
        rotation_quat = quaternion_from_euler(0, 0, 0)
        transform.transform.rotation.x = rotation_quat[0]
        transform.transform.rotation.y = rotation_quat[1]
        transform.transform.rotation.z = rotation_quat[2]
        transform.transform.rotation.w = rotation_quat[3]
        self.tf_broadcaster.sendTransform(transform)

def main():
    rclpy.init()
    tf_node = DynamicTFBroadcaster()
    rclpy.spin(tf_node)
    rclpy.shutdown()

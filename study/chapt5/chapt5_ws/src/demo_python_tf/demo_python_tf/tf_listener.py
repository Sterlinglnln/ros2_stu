import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.timer = self.create_timer(1.0, self.get_transform)
    
    def get_transform(self):
        try:
            result = self.buffer.lookup_transform('base_link', 'bottle_link', rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1.0))
            translation = result.transform
            ratation_euler = euler_from_quaternion([
                result.transform.rotation.x,
                result.transform.rotation.y,
                result.transform.rotation.z,
                result.transform.rotation.w
            ])
            self.get_logger().info(f'平移: {translation.translation}, 旋转四元数: {translation.rotation}, 旋转欧拉角: {ratation_euler}')
        except Exception as e:
            self.get_logger().warn(f'无法获取变换，原因: {str(e)}')

def main():
    rclpy.init()
    node = TFListener()
    rclpy.spin(node)
    rclpy.shutdown()

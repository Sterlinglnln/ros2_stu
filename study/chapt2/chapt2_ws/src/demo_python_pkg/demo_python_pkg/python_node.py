import rclpy
from rclpy.node import Node

def main():
    rclpy.init()  # 初始化rclpy
    node = Node("python_node")  # 创建一个名为 python_node 的节点
    node.get_logger().info("Hello, ROS2 from Python!")  # 打印日志
    rclpy.spin(node)  # 保持节点运行，等待回调
    rclpy.shutdown()  # 关闭rclpy

if __name__=="__main__":
    main()

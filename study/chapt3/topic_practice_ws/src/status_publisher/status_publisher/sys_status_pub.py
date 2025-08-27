import rclpy
from rclpy.node import Node
from status_interfaces.msg import SystemStatus
import psutil
import platform
import time  # 用于获取人类可读时间

class SysStatusPublisher(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.status_publisher = self.create_publisher(
            SystemStatus, 'system_status', 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.get_logger().info('System status publisher started')
        
    def timer_callback(self):
        # 获取系统状态数据（确保数据有效性）
        cpu_percent = psutil.cpu_percent()
        memory_info = psutil.virtual_memory()
        net_io_counters = psutil.net_io_counters()
        
        # 验证关键数据是否合理
        if cpu_percent < 0 or cpu_percent > 100:
            self.get_logger().warn(f"异常CPU使用率: {cpu_percent}%")
        
        msg = SystemStatus()
        # 时间处理：使用标准时间格式（年-月-日 时:分:秒）
        current_time = time.localtime()
        msg.readable_stamp = time.strftime("%Y-%m-%d %H:%M:%S", current_time)
        msg.stamp = self.get_clock().now().to_msg()  # 保留ROS时间戳
        
        # 系统信息
        msg.host_name = platform.node()
        
        # 核心修复：显式类型转换，确保与.msg定义一致
        msg.cpu_percent = float(cpu_percent)  # 确保是float类型
        msg.memory_percent = float(memory_info.percent)
        
        # 内存单位转换（GB）并显式转为float
        msg.memory_total = float(memory_info.total / (1024**3))  # 1024^3 = GB
        msg.memory_available = float(memory_info.available / (1024**3))
        
        # 网络数据（MB）并显式转为float
        msg.net_sent = float(net_io_counters.bytes_sent / (1024**2))
        msg.net_recv = float(net_io_counters.bytes_recv / (1024**2))
        
        # 日志输出验证数据
        self.get_logger().info(
            f"CPU: {msg.cpu_percent}%, "
            f"内存: {msg.memory_percent}%, "
            f"总内存: {msg.memory_total:.2f}GB"
        )
        self.status_publisher.publish(msg)

def main(args = None):
    rclpy.init(args=args)
    node = SysStatusPublisher('sys_status_pub')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

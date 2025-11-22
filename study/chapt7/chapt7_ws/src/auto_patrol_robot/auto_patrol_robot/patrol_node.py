import rclpy
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformListener, Buffer
from tf_transformations import quaternion_from_euler
from rclpy.duration import Duration
from rclpy.time import Time
from auto_patrol_interfaces.srv import SpeachText
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class PatrolNode(BasicNavigator):
    def __init__(self, node_name='patrol_node'):
        super().__init__(node_name)
        # 导航相关定义
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        self.declare_parameter('target_points',[0.0, 0.0, 0.0, 1.0, 1.0, 1.57])
        init_val = self.get_parameter('initial_point').value
        if not isinstance(init_val, list) or len(init_val) < 3:
            self.get_logger().warn(
                'initial_point 参数格式错误，使用默认值 [0.0, 0.0, 0.0]')
            init_val = [0.0, 0.0, 0.0]
        self.initial_point_: list[float] = init_val
        tar_val = self.get_parameter('target_points').value
        if not isinstance(tar_val, list) or len(tar_val) < 3:
            self.get_logger().warn(
                'target_points 参数格式错误，使用默认值 [0.0, 0.0, 0.0]')
            tar_val = [0.0, 0.0, 0.0]
        self.target_points_: list[float] = tar_val

        # 订阅与保存图像相关定义
        self.declare_parameter('image_save_path', '')
        self.image_save_path_ = self.get_parameter('image_save_path').value
        self.bridge_ = CvBridge()
        self.latest_image = None
        self.subscription_image = self.create_subscription(
            Image,
            '/camera/depth_image_raw',
            self.depth_callback,
            10)
        self.latest_depth = None

        # TF 用于查询实时位姿
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # 语音合成客户端
        self.speach_client_ = self.create_client(SpeachText, 'speech_text')

    def get_pose_by_xyyaw(self, x, y, yaw):
        """
        通过x,y,yaw合成PoseStamped
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, float(yaw))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def init_robot_pose(self):
        """
        初始化机器人位姿
        """
        try:
            x, y, yaw = self.initial_point_
        except ValueError:
            self.get_logger().warn('initial_point 参数不合法，使用默认 (0,0,0)')
            x, y, yaw = 0.0, 0.0, 0.0
        initial_pose = self.get_pose_by_xyyaw(x, y, yaw)
        self.setInitialPose(initial_pose)
        # 等待 Nav2 lifecycle 就绪
        self.waitUntilNav2Active()
        self.get_logger().info(
            f'初始位姿已设置: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} (rad)')

    def get_target_points(self):
        """
        获取目标点集合
        """
        if len(self.target_points_) % 3 != 0:
            self.get_logger().warn('target_points 长度非3倍数，忽略多余数据')
        goals = []
        triple_count = len(self.target_points_) // 3
        for i in range(triple_count):
            x = self.target_points_[3 * i]
            y = self.target_points_[3 * i + 1]
            yaw = self.target_points_[3 * i + 2]
            goals.append(self.get_pose_by_xyyaw(x, y, yaw))
        self.get_logger().info(f'加载目标点 {len(goals)} 个')
        return goals

    def nav_to_pose(self, target_pose):
        """
        导航到指定位置
        """
        self.get_logger().info(
            f'开始导航: x={target_pose.pose.position.x:.2f}, '
            f'y={target_pose.pose.position.y:.2f}'
        )
        self.goToPose(target_pose)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback is not None:
                remaining = Duration.from_msg(
                    feedback.estimated_time_remaining).nanoseconds / 1e9
                self.get_logger().info(f'预计剩余 {remaining:.1f} 秒')
                # 超时保护
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=300):
                    self.get_logger().warn('导航超时，取消任务')
                    self.cancelTask()
                    break

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('导航成功')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('导航被取消')
        elif result == TaskResult.FAILED:
            self.get_logger().error('导航失败')
        else:
            self.get_logger().error('导航结果未知')
        return result

    def get_current_pose(self):
        """
        通过tf获取机器人当前位姿
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'获取当前位姿失败: {exc}')
            return None

        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z

        pose.orientation = transform.transform.rotation
        return pose
    
    def speach_text(self, text):
        """
        调用服务播报语音
        """
        while not self.speach_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待语音服务...')
        request = SpeachText.Request()
        request.text = text
        future = self.speach_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            result = future.result().result
            if result:
                self.get_logger().info(f'语音播报成功: {text}')
            else:
                self.get_logger().warn(f'语音播报失败: {text}')
        else:
            self.get_logger().warn('调用语音服务失败')

    def depth_callback(self, msg):
        """
        将最新的消息放到 latest_image 变量中
        """
        self.latest_depth = msg

    def record_depth_image(self):
        if self.latest_depth is None:
            self.get_logger().warn("No depth image received yet, skip saving.")
            return

        try:
            # 正确的深度图转换（保持16bit）
            cv_depth = self.bridge_.imgmsg_to_cv2(
                self.latest_depth,
                desired_encoding='passthrough'
            )

            # 保存为 PNG (16-bit)
            timestamp = int(self.get_clock().now().nanoseconds / 1e6)
            filename = f"/home/larry/depth_{timestamp}.png"

            cv2.imwrite(filename, cv_depth)

            self.get_logger().info(f"深度图已保存: {filename}")

        except Exception as e:
            self.get_logger().error(f"保存深度图失败: {e}")

def main():
    rclpy.init()
    navigator = PatrolNode()

    # 初始化语音播报并获取目标点
    navigator.speach_text('Initializing patrol robot position')
    navigator.init_robot_pose()
    navigator.speach_text('Position initialization complete')
    goal_poses = navigator.get_target_points()
    if not goal_poses:
        navigator.get_logger().error('No valid target points configured, exiting patrol')
        navigator.destroy_node()
        rclpy.shutdown()
        return

    # 逐点巡逻
    for idx, goal in enumerate(goal_poses, start=1):
        navigator.get_logger().info(f'==> 巡逻点 {idx}/{len(goal_poses)}')
        result = navigator.nav_to_pose(goal)
        # 记录图像
        navigator.speach_text(text=f"Arrived at patrol point, recording image.")
        navigator.record_depth_image()
        navigator.speach_text(text=f"Recording complete.")

        if result != TaskResult.SUCCEEDED:
            navigator.get_logger().warn('巡逻中断')
            break

    navigator.get_logger().info('巡逻任务结束')
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

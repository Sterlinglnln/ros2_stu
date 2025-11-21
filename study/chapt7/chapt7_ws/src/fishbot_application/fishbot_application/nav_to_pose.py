from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():
    rclpy.init()

    navigator = BasicNavigator()
    # Wait until Nav2 is active
    navigator.waitUntilNav2Active()
    # Define the goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 1.0
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.orientation.w = 1.0
    # send the goal pose to the navigator and receive the result
    navigator.goToPose(goal_pose)
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        navigator.get_logger().info(f'预计：{
            Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} 秒到达目标位置')
        # 超时自动取消
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=300):
            navigator.cancelTask()
    
    # 获取任务结果
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('导航成功！')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().info('导航被取消！')
    elif result == TaskResult.FAILED:
        navigator.get_logger().info('导航失败！')
    else:
        navigator.get_logger().info('导航结果:返回状态无效！')
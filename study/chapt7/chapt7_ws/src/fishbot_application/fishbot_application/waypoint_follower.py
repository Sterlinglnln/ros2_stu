from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():
    rclpy.init()

    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()
    
    # 定义3个路点（与原代码一致）
    goal_poses = []
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 0.0
    goal_pose1.pose.position.y = 0.0
    goal_pose1.pose.orientation.w = 1.0
    goal_poses.append(goal_pose1)

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 2.0
    goal_pose2.pose.position.y = 0.0
    goal_pose2.pose.orientation.w = 1.0
    goal_poses.append(goal_pose2)

    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = 2.0
    goal_pose3.pose.position.y = 2.0
    goal_pose3.pose.orientation.w = 1.0
    goal_poses.append(goal_pose3)
    
    # 调用路点导航接口
    navigator.goThroughPoses(goal_poses)
    
    # 记录已完成的路点数量（初始为0）
    completed_waypoints = 0
    # 导航循环：监控路点进度
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback is not None:
            # 1. 获取当前正在执行的路点索引（兼容新旧版本）
            if hasattr(feedback, 'current_waypoint_index'):
                current_idx = feedback.current_waypoint_index
            elif hasattr(feedback, 'current_waypoint'):
                current_idx = feedback.current_waypoint
            else:
                current_idx = 0

            # 2. 检测是否完成了一个新的路点（索引递增时触发）
            if current_idx > completed_waypoints:
                completed_waypoints = current_idx
                navigator.get_logger().info(
                    f'✅ 路点 {completed_waypoints}/{len(goal_poses)} 完成！')

            # 3. 显示当前导航进度（不依赖距离计算，兼容旧版本）
            navigator.get_logger().info(
                f'📍 当前进度：正在导航到路点 {current_idx + 1}/{len(goal_poses)}'
            )

    # 所有路点完成后的最终结果
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('🎉 所有路点导航全部成功完成！')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().info('❌ 路点导航被用户取消！')
    elif result == TaskResult.FAILED:
        navigator.get_logger().info('❌ 路点导航失败！')
    else:
        navigator.get_logger().info('⚠️  路点导航结果：无效状态！')
    
    # 释放资源
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
import os

# 生成启动描述
def generate_launch_description():
    # 获取功能包路径
    fishbot_desc_path = get_package_share_path("fishbot_description")
    urdf_path = os.path.join(fishbot_desc_path, "urdf", "fishbot", "fishbot.urdf.xacro")
    rviz_config_path = os.path.join(fishbot_desc_path, "config", "display_robot_model.rviz")
    fishbot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    fishbot_controllers = os.path.join(fishbot_desc_path, "config", "fishbot_controllers.yaml")

    # 机器人状态发布节点
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": fishbot_description}],
    )

    # 控制器管理节点
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[fishbot_controllers],
    )

    # joint_state_broadcaster 控制器加载节点
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # diff_drive 控制器加载节点
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    # rviz2 可视化节点
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
        rviz_node,
    ])
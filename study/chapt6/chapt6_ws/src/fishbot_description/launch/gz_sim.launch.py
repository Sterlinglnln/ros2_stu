import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 获取功能包路径
    fishbot_desc_path = get_package_share_directory("fishbot_description")

    # 相关文件路径
    urdf_path = os.path.join(fishbot_desc_path, "urdf", "fishbot", "fishbot.urdf.xacro")
    rviz_config_path = os.path.join(fishbot_desc_path, "config", "display_robot_model.rviz")
    gazebo_config_path = os.path.join(fishbot_desc_path, "config", "ros_gz_bridge.yaml")
    custom_world_path = os.path.join(fishbot_desc_path, "worlds", "test_world.sdf")

    # 机器人状态发布节点
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(
                Command(["xacro ", urdf_path]),
                value_type=str,
            )
        }],
    )

    # Gazebo 仿真环境启动
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": f"{custom_world_path} -r"}.items(),
    )

    # 生成并发布机器人模型到 Gazebo
    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description"],
    )

    # ROS-Gazebo 桥接节点
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": gazebo_config_path}],
    )

    # RViz2 可视化节点
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    laser_frame_fix_node = Node(
        package='fishbot_description',
        executable='laser_frame_fix.py',
        name='laser_frame_fixer'
    )  

    # 组装所有启动项
    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot_node,
        bridge_node,
        rviz_node,
        laser_frame_fix_node
    ])

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # 获取功能包路径
    fishbot_desc_path = get_package_share_directory("fishbot_description")
    
    # 定义文件路径变量（与原XML的let变量完全对应）
    urdf_path = os.path.join(fishbot_desc_path, "urdf", "fishbot", "fishbot.urdf.xacro")
    rviz_config_path = os.path.join(fishbot_desc_path, "config", "display_robot_model.rviz")
    gazebo_config_path = os.path.join(fishbot_desc_path, "config", "ros_gz_bridge.yaml")
    
    # 1. 机器人状态发布节点
    robot_state_pub_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": launch_ros.parameter_descriptions.ParameterValue(
                launch.substitutions.Command(["xacro ", urdf_path]),
                value_type=str
            )
        }]
    )
    
    # 2. 启动Gazebo仿真
    gz_sim_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": "empty.sdf -r"
        }.items()
    )
    
    # 3. 在Gazebo中生成机器人模型
    spawn_robot_node = launch_ros.actions.Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description"],
        output="screen"
    )
    
    # 4. ROS-Gazebo桥接节点
    bridge_node = launch_ros.actions.Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": gazebo_config_path
        }]
    )

    # 5. 启动RViz2
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path]  # 保持原配置路径
    )
    
    # 组装所有启动项（与原XML节点顺序一致）
    return launch.LaunchDescription([
        robot_state_pub_node,
        gz_sim_launch,
        spawn_robot_node,
        bridge_node,
        rviz_node
    ])

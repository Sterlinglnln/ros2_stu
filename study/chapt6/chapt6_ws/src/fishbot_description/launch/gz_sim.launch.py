import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_fishbot_description = get_package_share_directory('fishbot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # 定义文件路径
    world_path = os.path.join(pkg_fishbot_description, 'worlds', 'model.sdf')
    xacro_path = os.path.join(pkg_fishbot_description, 'urdf', 'fishbot', 'fishbot.urdf.xacro')
    
    # 启动Gazebo仿真环境
    gazebo_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        # 传递世界文件参数
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # 解析xacro文件，生成机器人描述
    robot_model_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            # 使用Command调用xacro命令
            'robot_description': Command(['xacro ', xacro_path]),
            'use_sim_time': True
        }]
    )
    
    # 发布关节状态
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )
    
    # 将机器人模型生成到Gazebo中
    spawn_fishbot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'fishbot',
            '-topic', 'robot_description',
            '-x', '1', '-y', '0', '-z', '0.1'
            ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_start,
        robot_model_publisher,
        joint_state_publisher,
        spawn_fishbot
    ])

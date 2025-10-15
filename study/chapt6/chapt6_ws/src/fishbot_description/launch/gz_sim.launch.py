import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # 获取默认路径
    robot_name_in_model = "fishbot"
    urdf_tutorial_path = get_package_share_directory("fishbot_description")
    default_model_path = os.path.join(urdf_tutorial_path, 'urdf', 'fishbot', 'fishbot.urdf.xacro')
    default_world_path = os.path.join(urdf_tutorial_path, 'worlds', 'empty.sdf')
    
    # 控制器配置文件路径
    controllers_config_path = os.path.join(urdf_tutorial_path, 'config', 'controllers.yaml')

    # 为launch声明参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF的绝对路径')
    
    action_declare_arg_world_path = launch.actions.DeclareLaunchArgument(
        name='world', default_value=str(default_world_path),
        description='世界文件的绝对路径')
    
    # 获取文件内容生成新的参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 通过 IncludeLaunchDescription 引入ros_gz_sim的launch文件
    launch_gz_sim = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
        # 加载世界文件
        launch_arguments={
            'gz_args': [launch.substitutions.LaunchConfiguration('world')],
            'use_sim_time': 'true'
        }.items()
    )
    
    # 请求加载机器人模型
    spawn_entity_node = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', robot_name_in_model,
                   '-topic', 'robot_description'],
        output='screen'
    )
    
    # 控制器管理器节点
    controller_manager_node = launch_ros.actions.Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},
                    controllers_config_path],
        output='screen'
    )
    
    # 激活差速控制器
    activate_diff_drive_spawner = launch_ros.actions.Node(
        cmd=['ros2', 'control', 'set_controller_state', 'fishbot_base_controller', 'active'],
        output='screen'
    )
    
    # 键盘遥控节点
    teleop_node = launch_ros.actions.Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        parameters=[{'use_sim_time': True}]
    )
    
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_declare_arg_world_path,
        robot_state_publisher_node,
        launch_gz_sim,
        spawn_entity_node,
        controller_manager_node,
        activate_diff_drive_spawner,
        teleop_node
    ])
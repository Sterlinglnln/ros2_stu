import launch
import launch_ros

def generate_launch_description():
    # 创建参数说明action，用于解析launch命令后的参数
    action_declare_arg_max_speed = launch.actions.DeclareLaunchArgument('launch_max_speed', default_value='2.0')
    action_node_turtle_control = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable='turtle_control',
        # 将参数传递给节点
        parameters=[{'max_speed': launch.substitutions.LaunchConfiguration('launch_max_speed', default='2.0')}],
        output='screen',
    )
    action_node_patrol_client = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable='patrol_client',
        output='log',
    )
    action_node_turtlesim_node = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        output='both',
    )
    # 启动所有节点并且返回
    launch_description = launch.LaunchDescription([
        action_declare_arg_max_speed,
        action_node_turtle_control,
        action_node_patrol_client,
        action_node_turtlesim_node,
    ])
    return launch_description
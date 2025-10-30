import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包的共享目录
    pkg_share = get_package_share_directory('fishbot_description')
    
    # 定义XACRO文件路径
    default_model_path = launch.substitutions.PathJoinSubstitution(
        [pkg_share, 'urdf', 'fishbot', 'fishbot.urdf.xacro']
    )

    # 定义RViz配置文件路径
    default_rviz_config_path = launch.substitutions.PathJoinSubstitution(
        [pkg_share, 'config', 'display_robot_model.rviz']
    )
    
    # 声明模型路径参数
    action_declare_model_path = launch.actions.DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path,
        description='XACRO 的绝对路径'
    )
    
    # 声明RViz配置路径参数
    action_declare_rviz_config_path = launch.actions.DeclareLaunchArgument(
        name='rviz_config',
        default_value=default_rviz_config_path,
        description='RViz配置文件的路径'
    )
    
    # 构建机器人描述参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]
        ),
        value_type=str
    )
    
    # 机器人状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    # 关节状态发布节点
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
    
    # RViz2节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', launch.substitutions.LaunchConfiguration('rviz_config')]
    )
    
    return launch.LaunchDescription([
        action_declare_model_path,
        action_declare_rviz_config_path,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取默认路径
    robot_name_in_model = 'fishbot'
    urdf_tutorial_path = get_package_share_directory('fishbot_description')
    default_model_path = urdf_tutorial_path + '/urdf/fishbot/fishbot.urdf.xacro'
    default_world_path = urdf_tutorial_path + '/world/room/model.sdf'

    # 为 launch 文件声明参数
    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=default_model_path,
        description='Absolute path to robot urdf file')
    
    # 处理机器人描述文件
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 启动 Gazebo 模拟器 - 修正包路径
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'ros_gz_sim'), '/launch', '/gz_sim.launch.py']),  # 修正为ros_gz_sim包
        launch_arguments=[('world', default_world_path), ('verbose', 'true')]
    )
    
    # 启动机器人模型 - 修正spawn节点的包和可执行文件
    spawn_entity_node = launch_ros.actions.Node(
        package='ros_gz_sim',  # 修正为ros_gz_sim包
        executable='create',  # 修正为create可执行文件
        arguments=['-topic', 'robot_description',
                   '-name', robot_name_in_model,  # 注意参数名是-name而非-entity
                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],  # 可选：添加初始位置
    )

    return launch.LaunchDescription([
        action_declare_arg_model_path,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node
    ])

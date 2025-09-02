import launch
import launch_ros

def generate_launch_description():
    # 创建节点
    face_detect_node = launch_ros.actions.Node(
        package='demo_python_service',
        executable='face_detect_node',
        name='face_detect_node'
    )
    face_detect_client_node = launch_ros.actions.Node(
        package='demo_python_service',
        executable='face_detect_client_node',
        name='face_detect_client_node'
    )
    # 合成启动描述并返回
    launch_description = launch.LaunchDescription([
        face_detect_node,
        face_detect_client_node
    ])
    return launch_description

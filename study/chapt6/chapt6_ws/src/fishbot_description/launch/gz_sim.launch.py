#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import tempfile
import subprocess
from textwrap import dedent

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _gen_and_spawn(context):
    # 获取参数
    world = LaunchConfiguration("world").perform(context)
    model_name = LaunchConfiguration("model_name").perform(context)
    xacro_file = LaunchConfiguration("xacro_file").perform(context)
    left_joint = LaunchConfiguration("left_joint").perform(context)
    right_joint = LaunchConfiguration("right_joint").perform(context)
    wheel_separation = LaunchConfiguration("wheel_separation").perform(context)
    wheel_radius = LaunchConfiguration("wheel_radius").perform(context)
    odom_frame = LaunchConfiguration("odom_frame").perform(context)
    base_frame = LaunchConfiguration("base_frame").perform(context)
    odom_freq = LaunchConfiguration("odom_publish_frequency").perform(context)
    max_lin = LaunchConfiguration("max_linear_speed").perform(context)
    max_ang = LaunchConfiguration("max_angular_speed").perform(context)

    # 1) 运行 xacro -> URDF
    try:
        urdf_xml = subprocess.run(
            ["xacro", xacro_file],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"[gz_sim.launch.py] xacro 生成 URDF 失败: {e.stderr}") from e

    # 2) URDF -> SDF：写入临时 URDF 文件，再调用 `gz sdf -p <file>`
    try:
        with tempfile.NamedTemporaryFile(prefix=f"{model_name}_", suffix=".urdf", delete=False, mode="w", encoding="utf-8") as tmp_urdf:
            tmp_urdf_path = tmp_urdf.name
            tmp_urdf.write(urdf_xml)

        sdf_xml = subprocess.run(
            ["gz", "sdf", "-p", tmp_urdf_path],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"[gz_sim.launch.py] URDF 转 SDF 失败: {e.stderr}") from e
    finally:
        # 清理临时 URDF
        try:
            os.remove(tmp_urdf_path)
        except Exception:
            pass

    # 3) 注入 Gazebo Sim DiffDrive 插件
    plugin_xml = f"""
      <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>{left_joint}</left_joint>
        <right_joint>{right_joint}</right_joint>
        <wheel_separation>{wheel_separation}</wheel_separation>
        <wheel_radius>{wheel_radius}</wheel_radius>

        <topic>cmd_vel</topic>
        <odom_frame>{odom_frame}</odom_frame>
        <robot_base_frame>{base_frame}</robot_base_frame>
        <odom_publish_frequency>{odom_freq}</odom_publish_frequency>
        <odom_source>world</odom_source>
        <max_linear_speed>{max_lin}</max_linear_speed>
        <max_angular_speed>{max_ang}</max_angular_speed>
      </plugin>
    """.rstrip()

    # 简单插入到第一个 </model> 之前
    insert_pos = sdf_xml.rfind("</model>")
    if insert_pos == -1:
        raise RuntimeError("[gz_sim.launch.py] 未在 SDF 中找到 <model>，请检查 xacro/URDF 是否有效。")

    sdf_with_plugin = sdf_xml[:insert_pos] + plugin_xml + sdf_xml[insert_pos:]

    # 4) 写临时 SDF
    tmp_sdf = tempfile.NamedTemporaryFile(prefix=f"{model_name}_", suffix=".sdf", delete=False)
    tmp_sdf_path = tmp_sdf.name
    with open(tmp_sdf_path, "w", encoding="utf-8") as f:
        f.write(sdf_with_plugin)

    # 5) 启动进程/节点
    actions = []

    # Gazebo Sim
    actions.append(
        ExecuteProcess(
            cmd=["gz", "sim", "-r", world],
            output="screen"
        )
    )

    # 生成模型
    actions.append(
        Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_fishbot",
            output="screen",
            arguments=["-name", model_name, "-file", tmp_sdf_path]
        )
    )

    # 桥接
    actions.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            output="screen",
            arguments=[
                f"/cmd_vel@geometry_msgs/msg/Twist@/model/{model_name}/cmd_vel@gz.msgs.Twist",
                f"/odometry@nav_msgs/msg/Odometry@/model/{model_name}/odometry@gz.msgs.Odometry",
            ],
            parameters=[{"use_sim_time": True}],
        )
    )

    # robot_state_publisher（直接用原始 URDF）
    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": urdf_xml, "use_sim_time": True}],
        )
    )

    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory("fishbot_description")
    default_xacro = os.path.join(pkg_share, "urdf", "fishbot", "fishbot.urdf.xacro")

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value="empty.sdf",
            description="Gazebo Sim world file (absolute path or resolvable via GZ_RESOURCE_PATH)"
        ),
        DeclareLaunchArgument(
            "model_name",
            default_value="fishbot",
            description="Model name inside Gazebo Sim"
        ),
        DeclareLaunchArgument(
            "xacro_file",
            default_value=default_xacro,
            description="Path to fishbot.urdf.xacro"
        ),
        # 关键差速参数与关节名（按你的 xacro 实际命名覆盖）
        DeclareLaunchArgument("left_joint", default_value="left_wheel_joint"),
        DeclareLaunchArgument("right_joint", default_value="right_wheel_joint"),
        DeclareLaunchArgument("wheel_separation", default_value="0.16"),
        DeclareLaunchArgument("wheel_radius", default_value="0.032"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("odom_publish_frequency", default_value="50"),
        DeclareLaunchArgument("max_linear_speed", default_value="2.0"),
        DeclareLaunchArgument("max_angular_speed", default_value="6.0"),

        OpaqueFunction(function=_gen_and_spawn),
    ])
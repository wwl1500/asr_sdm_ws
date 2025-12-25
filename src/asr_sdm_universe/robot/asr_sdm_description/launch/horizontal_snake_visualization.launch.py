#!/usr/bin/env python3
"""
水平面蛇形机器人可视化 Launch 文件

启动：
- robot_state_publisher: 发布机器人模型
- RViz2: 3D 可视化

使用方法：
ros2 launch asr_sdm_description horizontal_snake_visualization.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 路径
    pkg_share = FindPackageShare("asr_sdm_description")
    default_model = PathJoinSubstitution([pkg_share, "urdf", "horizontal_snakerobot.urdf.xacro"])
    default_rviz = PathJoinSubstitution([pkg_share, "rviz", "horizontal_snake.rviz"])

    # 参数
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=default_model,
        description="水平面蛇形机器人 URDF 文件路径",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz,
        description="RViz 配置文件路径",
    )

    # robot_description - 使用 ParameterValue 包装
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            LaunchConfiguration("model"),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # robot_state_publisher 节点
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # RViz2 节点
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
    )

    return LaunchDescription(
        [
            model_arg,
            rviz_arg,
            robot_state_publisher_node,
            rviz_node,
        ]
    )

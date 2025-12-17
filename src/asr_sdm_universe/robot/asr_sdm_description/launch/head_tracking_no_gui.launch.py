from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable


def generate_launch_description():
    # 路径
    pkg_share = FindPackageShare("asr_sdm_description")
    default_model = PathJoinSubstitution([pkg_share, "urdf", "underwater_snakerobot.urdf.xacro"])
    default_rviz = PathJoinSubstitution([pkg_share, "rviz", "head_tracking.rviz"])

    # 参数
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=default_model,
        description="Absolute path to the robot xacro/URDF file.",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz,
        description="RViz config file",
    )

    # robot_description
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            LaunchConfiguration("model"),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description],
    )

    # 不启动 joint_state_publisher_gui，避免覆盖控制器
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


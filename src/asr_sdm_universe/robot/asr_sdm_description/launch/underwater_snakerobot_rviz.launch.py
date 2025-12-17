from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 包路径和默认 URDF（xacro）
    pkg_share = FindPackageShare("asr_sdm_description")
    default_model = PathJoinSubstitution([pkg_share, "urdf", "underwater_snakerobot.urdf.xacro"])

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=default_model,
        description="Absolute path to the robot xacro/URDF file.",
    )

    # 用 xacro 生成 robot_description
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            LaunchConfiguration("model"),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 只启 GUI 版关节发布器，方便手动调姿态
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description],
    )

    # 启动一个“干净”的 RViz 2，不加载旧的 RViz 配置
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
    )

    return LaunchDescription(
        [
            model_arg,
            joint_state_publisher_gui_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
    Command,
    FindExecutable,
    PythonExpression,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config_file = (
        PathJoinSubstitution(
            [
                FindPackageShare("coordinated_motion_examples"),
                "config",
                "coordinated_motion_examples.rviz",
            ]
        ),
    )

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            default_value="robot6R",
            choices=["robot6R", "robot7R"],
            description="Select which robot configuration to use",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller",
            choices=[
                "coordinated_pose_controller",
                "coordinated_as_controller",
            ],
            default_value="coordinated_as_controller",
            description="Which controller should be started?",
        )
    )

    robot_type = LaunchConfiguration("robot_type")
    controller = LaunchConfiguration("controller")

    pkg_share = FindPackageShare("coordinated_motion_examples")
    controller_config = PathJoinSubstitution(
        [
            pkg_share,
            "config",
            PythonExpression(['"', robot_type, '" + "_controllers.yaml"']),
        ]
    )
    description_file = PathJoinSubstitution(
        [
            pkg_share,
            "urdf",
            PythonExpression(['"two_" + "', robot_type, '" + ".xacro"']),
        ]
    )

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controller_config,
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            [TextSubstitution(text="rob1_"), controller],
            [TextSubstitution(text="rob2_"), controller],
            "--controller-manager",
            "controller_manager",
        ],
    )

    positioner_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "positioner_controller",
            "--controller-manager",
            "controller_manager",
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        positioner_controller_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)

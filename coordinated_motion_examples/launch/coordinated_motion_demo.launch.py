from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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
            default_value="coordinated_pose_controller",
            description="Which controller should be started?",
        )
    )

    robot_type = LaunchConfiguration("robot_type")
    controller = LaunchConfiguration("controller")

    robot_demo_nodes = []
    for arm_id in ["rob1", "rob2"]:
        robot_demo_nodes.append(
            Node(
                package="coordinated_motion_examples",
                executable="coordinated_motion_demo_" + arm_id,
                name="coordinated_motion_client",
                output="screen",
                parameters=[
                    {
                        "controller": [arm_id, "_", controller],
                        "robot_type": robot_type,
                        "arm_id": arm_id,
                    }
                ],
            )
        )
    return LaunchDescription(declared_arguments + robot_demo_nodes)

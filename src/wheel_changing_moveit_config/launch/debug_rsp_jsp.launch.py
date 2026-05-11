from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def clean(obj):
    if isinstance(obj, tuple):
        return list(obj)
    if isinstance(obj, dict):
        return {k: clean(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [clean(v) for v in obj]
    return obj


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "ur10e_wheel_changer",
            package_name="wheel_changing_moveit_config",
        )
        .to_moveit_configs()
    )

    robot_description = clean(moveit_config.robot_description)

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
            parameters=[robot_description],
        ),
    ])

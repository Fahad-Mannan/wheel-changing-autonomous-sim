from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def clean(obj):
    """Convert any accidental tuple values into lists so ROS launch accepts them."""
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
    robot_description_semantic = clean(moveit_config.robot_description_semantic)
    robot_description_kinematics = clean(moveit_config.robot_description_kinematics)
    joint_limits = clean(moveit_config.joint_limits)
    planning_pipelines = clean(moveit_config.planning_pipelines)
    trajectory_execution = clean(moveit_config.trajectory_execution)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": False},
        ],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": False},
        ],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            planning_pipelines,
            trajectory_execution,
            {"use_sim_time": False},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipelines,
            joint_limits,
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        move_group,
        rviz,
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "ur10e_wheel_changer",
            package_name="wheel_changing_moveit_config",
        )
        .to_moveit_configs()
    )

    ompl_only = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath",
            ],
            "planner_configs": {
                "RRTConnectkConfigDefault": {
                    "type": "geometric::RRTConnect",
                    "range": 0.0,
                }
            },
            "ur_manipulator": {
                "default_planner_config": "RRTConnectkConfigDefault",
                "planner_configs": ["RRTConnectkConfigDefault"],
                "projection_evaluator": "joints(shoulder_pan_joint,shoulder_lift_joint)",
                "longest_valid_segment_fraction": 0.005,
            },
        },
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": False},
        ],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": False},
        ],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            ompl_only,
            {"use_sim_time": False},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            ompl_only,
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        move_group,
        rviz,
    ])

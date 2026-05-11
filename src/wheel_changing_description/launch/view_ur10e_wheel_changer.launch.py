from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = PathJoinSubstitution([
        FindPackageShare('wheel_changing_description'),
        'urdf',
        'ur10e_wheel_changer.urdf.xacro'
    ])

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file, ' ur_type:=ur10e']),
            value_type=str
        )
    }

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),
    ])

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    world_file = PathJoinSubstitution([
        FindPackageShare('wheel_changing_gazebo'),
        'worlds',
        'wheel_changing_world.sdf'
    ])

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

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ur10e_wheel_changer',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0'
        ],
        output='screen'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        ),
        robot_state_publisher_node,
        spawn_robot_node,
    ])

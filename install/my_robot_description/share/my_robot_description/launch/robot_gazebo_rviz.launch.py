from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.actions import ExecuteProcess
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_path = get_package_share_directory('my_robot_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    return LaunchDescription([

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description
            }]
        ),

        # Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'four_wheel_bot'],
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])

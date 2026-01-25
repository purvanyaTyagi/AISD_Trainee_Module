import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Get the path to the URDF file
    package_name = 'my_robot_description' # Make sure this matches your package name
    urdf_file_name = 'robot.urdf'

    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file_name)

    # 2. Read the URDF file content
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 3. Robot State Publisher Node
    # This publishes the robot structure to the topic /robot_description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 4. Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # 5. Spawn the Robot
    # We tell Gazebo to look at the topic /robot_description to find our robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_custom_robot'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
    ])

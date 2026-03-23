from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. Paths
    pkg_path = get_package_share_directory('rover_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    urdf_file = os.path.join(pkg_path, 'urdf', 'rover.urdf')

    # 2. Read URDF content for robot_state_publisher
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 3. Launch Gazebo Fortress
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 4. Robot State Publisher (Provides /robot_description topic)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen'
    )

    # 5. Spawn Robot from URDF String
    # Note: Use '-topic' to pull the description directly from the publisher
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rover',
            '-topic', 'robot_description', # Uses the string from Node 4
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot
    ])

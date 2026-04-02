from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # paths
    pkg_rover = get_package_share_directory('rover_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_file = os.path.join(pkg_rover, 'urdf', 'rover.urdf')
    world_file = os.path.join(pkg_rover, 'world', 'world.sdf')

    obstacle_file = os.path.join(pkg_rover, 'world', 'obstacles.sdf')

    # start gazebo (changed for gazebo fortress)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        # uncomment here to use the lunar terrain
        launch_arguments={'gz_args': f'-r {world_file}'
        }.items(),

        # uncomment here to spawn in empty world
        #launch_arguments={'gz_args': '-r empty.sdf'
        #}.items(),
    )

    # spawn rover
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rover',
            '-file', urdf_file,
            '-x', '0',
            '-y', '0',
            '-z', '1.0' # add some height so the rover doesn't spawn inside the terrain
        ],
        output='screen'
    )

    # delay the spawn since sometimes it executes funny
    spawn_robot_delayed = TimerAction(
        period=2.0,
        actions=[spawn_robot]
    )

    # add LiDAR bridge
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        spawn_robot_delayed,
        lidar_bridge 
    ])
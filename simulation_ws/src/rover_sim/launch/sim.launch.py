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
    sdf_file = os.path.join(pkg_rover, 'rover', 'rover_model.sdf')

    
    # adding a state published to /odom topic for rviz visualization
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc
        }]
    )

    # start gazebo fortress 
    # NOTE: fortress uses gz_sim, classic uses gz or gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # spawn rover
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rover',
            '-file', sdf_file,

            # get x and y converted from the path planner and tiff file size 
            '-x', '-15.0', # X = (start_x - tiff_width/2)*10
            '-y', '10.0', # Y = (tiff_height - start_y)*10
            '-z', '20.0' # add some height if the rover spawns inside the terrain
        ],
        output='screen'
    )

    # expanded bridge to add tf and clock 
    # this maps rover pose to the /tf topic
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/model/rover/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/world/default/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        output='screen'
    )

    # static transform for fixed 'map' frame
    # links the 'map' frame to the odom or rover frame
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'rover/odom'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        TimerAction(period=2.0, actions=[spawn_robot]),
        bridge,
        static_tf
    ])

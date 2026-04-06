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
    
    # Read URDF for Robot State Publisher
    with open(sdf_file, 'r') as infp:
        robot_description_config = infp.read()

    # 1. Start Gazebo Fortress
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 2. Robot State Publisher (Broadcasts TFs for RViz)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    # 3. Spawn Rover
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rover',
            '-file', sdf_file,
            '-x', '0', '-y', '0', '-z', '1.0'
        ],
        output='screen'
    )

    # 4. Expanded Bridge (LiDAR + TF + Joint States)
    # Note: 'ignition' in the bridge string is used for Fortress; 
    # 'gz' is used for newer versions (Harmonic+).
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/model/rover/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/world/lunar_world/model/rover/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        remappings=[
            ('/model/rover/pose', '/tf'),
            ('/world/lunar_world/model/rover/joint_state', '/joint_states')
        ],
        output='screen'
    )

    # 5. RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # Optional: Add arguments=['-d', rviz_config_path] if you have a saved config
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        TimerAction(period=2.0, actions=[spawn_robot]),
        bridge,
        rviz
    ])

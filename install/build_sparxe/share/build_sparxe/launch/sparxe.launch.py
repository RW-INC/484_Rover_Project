from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('phidgets_spatial'),
                    'launch',
                    'spatial-launch.py',
                ])
            ),
            launch_arguments={'hub_port': '1'}.items()
        ),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'use_mag': True,
                'publish_tf': True,
                'world_frame': 'enu',
            }],
            remappings=[
                ('imu/data_raw', '/imu/data_raw'),
                ('imu/mag', '/imu/mag'),
                ('imu/data', '/imu/data')
            ]
        ),
        # Node(
        #     package='camera',
        #     executable='camera_stream_node',
        #     name='camera',
        # ),
        Node(
            package='status_updater',
            executable='status_updater_node',
            name='status_updater',
        ),
        Node(
            package='motor_commander',
            executable='motor_commander_node',
            name='motor_commander',
        ),
        Node(
            package='arduino_bridge',
            executable='arduino_bridge_node',
            name='arduino_bridge',
        ),
        Node(
            package='uwb_bridge',
            executable='uwb_bridge_node',
            name='uwb_bridge'
        )
    ])
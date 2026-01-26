import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    '''
    Launches the state estimation for the BLUEROV.

    :return: The launch description.
    '''
    sbg_config = os.path.join(
        '/home',
        'frostlab',
        'config',
        'sbg_driver_params.yaml'
    )
    ntrip_config = os.path.join(
        '/home',
        'frostlab',
        'config',
        'ntrip_client_params.yaml'
    )
    nmea_config = os.path.join(
        '/home',
        'frostlab',
        'config',
        'nmea_gpsd_params.yaml'
    )

    sim = "false"  # Default to 'false'
    GPS = "false"  # Default to 'false'
    verbose = "false"  # Default to 'false'
    param_file = '/home/frostlab/config/vehicle_params.yaml'

    if verbose == "true":
        output = 'screen'
    else:
        output = 'log'

    launch_actions = []

    launch_actions.extend([
        DeclareLaunchArgument('namespace', default_value='bluerov2'),
        launch_ros.actions.Node(
            package='sensor_bringup', 
            executable='dvl_converter', 
            name='dvl_to_twist_node',
            parameters=[param_file],
            namespace=LaunchConfiguration('namespace'),
        ),

        launch_ros.actions.Node(
            package='sensor_bringup',
            executable='gps_odom',
            name='gps_odom_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[param_file],
            remappings=[('fix', 'imu/nav_sat_fix')],
        ),
        launch_ros.actions.Node(
            package='sensor_bringup',
            executable='odom_anchor',
            name='imu_odom_anchor',
            namespace=LaunchConfiguration('namespace'),
            parameters=[param_file],
        ),
        launch_ros.actions.Node(
            package='sensor_bringup',
            executable='odom_anchor',
            name='dvl_odom_anchor',
            namespace=LaunchConfiguration('namespace'),
            parameters=[param_file],
        ),
        launch_ros.actions.Node(
            package='sensor_bringup',
            executable='dummy_imu_odom_name',
            name='dummy_imu_odom_name_node',
            namespace=LaunchConfiguration('namespace'),
        ),
        launch_ros.actions.Node(
            package='sensor_bringup',
            executable='odom_to_navsat',
            name='odom_to_navsat',
            namespace=LaunchConfiguration('namespace'),
            parameters=[param_file],
        ),
        launch_ros.actions.Node(
            package='sensor_bringup',
            executable='odom_to_navsat',
            name='dvl_to_navsat',
            namespace=LaunchConfiguration('namespace'),
            parameters=[param_file],
        ),



    ])

    return launch.LaunchDescription(launch_actions)
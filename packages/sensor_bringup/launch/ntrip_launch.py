import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
	ntrip_config = os.path.join(
        '/root',
        'config',
        'ntrip_client_params.yaml'
    )


	return LaunchDescription([
		DeclareLaunchArgument('namespace', default_value='/'),
		Node(
            package='ntrip_client',
            executable='ntrip_ros.py',
            name='ntrip_client',
            namespace=LaunchConfiguration('namespace'),
            parameters=[ntrip_config],
        ),
		
	])

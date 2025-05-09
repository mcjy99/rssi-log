#launches wifi_log and rosbag
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os
from datetime import datetime

def generate_launch_description():
    # Create timestamp for rosbag directory name
    timestamp = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
    
    # Launch arguments
    enable_recording = LaunchConfiguration('enable_recording')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'enable_recording',
            default_value='true',
            description='Enable/disable rosbag recording'
        ),
        
        # WiFi signal strength publisher
        Node(
            package='rssi-log',
            executable='wifi_log',
            name='wifi_log',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'publish_rate': 1.0}  # Rate in Hz (1 second interval)
            ]
        ),
        
        # MAVROS node
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'fcu_url': '/dev/ttyACM0:57600'},  # Adjust as needed for your setup udp://:14540@127.0.0.1:14557
                {'gcs_url': ''},
                {'target_system_id': 1},
                {'target_component_id': 1},
                {'fcu_protocol': 'v2.0'}
            ],
            remappings=[
                ('/mavros/local_position/odom', '/odom')  # Optional remapping if needed
            ]
        ),
        
        # Rosbag recorder for MAVROS odometry
        ExecuteProcess(
            condition=IfCondition(enable_recording),
            cmd=[
                'ros2', 'bag', 'record',
                '-o', f'mavros_odom_wifi_data_{timestamp}',
                '/mavros/local_position/odom',
                '/wifi_signal'
            ],
            output='screen'
        )
    ])
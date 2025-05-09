#launches lora_send and mavros and rosbag
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from datetime import datetime
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    sf_param = DeclareLaunchArgument(
        'sf',
        default_value='7',
        description='Spreading Factor for LoRa communication'
    )

    tp_param = DeclareLaunchArgument(
        'tp',
        default_value='22',
        description='Transmit power for LoRa'
    )
    
    fcu_url_param = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyACM0:57600',
        description='URL for MAVROS FCU connection'
    )
    
    gcs_url_param = DeclareLaunchArgument(
        'gcs_url',
        default_value='',
        description='URL for ground control station'
    )
    
    tgt_system_param = DeclareLaunchArgument(
        'tgt_system',
        default_value='1',
        description='Target system ID'
    )
    
    tgt_component_param = DeclareLaunchArgument(
        'tgt_component',
        default_value='1',
        description='Target component ID'
    )
    
    # Create unique filename for rosbag with timestamp
    current_time = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
    rosbag_dir = os.path.join(os.getcwd(), 'rosbags')
    rosbag_name = f'odom_data_{current_time}'
    
    # Make sure rosbag directory exists
    os.makedirs(rosbag_dir, exist_ok=True)
    
    # LoRa sender node
    lora_sender_node = Node(
        package='rssi-log',
        executable='lora_send',
        name='lora_send',
        parameters=[{
            'SF': LaunchConfiguration('sf')
            'TP':LaunchConfiguration('tp')
        }],
        output='screen'
    )
    
    # MAVROS node
    mavros_config_dir = get_package_share_directory('mavros')
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[
            {'fcu_url': LaunchConfiguration('fcu_url')},
            {'gcs_url': LaunchConfiguration('gcs_url')},
            {'target_system_id': LaunchConfiguration('tgt_system')},
            {'target_component_id': LaunchConfiguration('tgt_component')},
            # Default config files
            os.path.join(mavros_config_dir, 'launch', 'px4_config.yaml'),
            os.path.join(mavros_config_dir, 'launch', 'px4_pluginlists.yaml')
        ]
    )
    
    # ROS2 bag recording
    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record',
             '/mavros/local_position/odom',  # Record odometry data
             '-o', os.path.join(rosbag_dir, rosbag_name)],
        output='screen'
    )
    
    # Log information about recording
    log_info = LogInfo(
        msg=f"Recording odometry data to {os.path.join(rosbag_dir, rosbag_name)}"
    )
    
    return LaunchDescription([
        # Launch arguments
        sf_param,
        fcu_url_param,
        gcs_url_param,
        tgt_system_param,
        tgt_component_param,
        
        # Nodes and processes
        lora_sender_node,
        mavros_node,
        log_info,
        rosbag_record
    ])
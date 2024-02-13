
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import math
import os
from ament_index_python.packages import get_package_share_directory
    
def generate_launch_description():
    # Declare the launch arguments
    min_height_for_move_ = LaunchConfiguration('min_height_for_move')
    min_height_for_move_arg = DeclareLaunchArgument(
        'min_height_for_move',
        default_value="0.05",
        description='Top-level namespace')


    # sim_ = LaunchConfiguration('use_sim_time')
    # use_sim_time_arg = DeclareLaunchArgument(
    #     'use_sim_time',
    #     default_value="False",
    #     description='use_sim_time')
    # if sim_ == "true" or sim_ ==  "True":
    #     use_sim_time_ = True
    # else:
    #     use_sim_time_ = False
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Whether to use simulation time or not'
    )
    theta_min_deg = -60
    theta_max_deg = 60

    theta_min_rad = theta_min_deg * math.pi / 180
    theta_max_rad = theta_max_deg * math.pi / 180
    fake_scan_move = Node(
        package='fake_frame',
        executable='fake_scan',
        name='fake_utliadr_scan',
            parameters=[{'target_topic': "utlidar_scan_for_move"}]    
    )

    return LaunchDescription([
        min_height_for_move_arg,
        use_sim_time_arg,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='utlidar_to_fake_utlidar',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'utlidar_lidar', 'fake_utlidar_lidar']
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/filtered/utlidar/cloud'),
                        ('scan', '/utlidar_scan_for_move')],
            parameters=[{
                'min_height': min_height_for_move_,
                'max_height': 1.0,
                'angle_min': -math.pi,
                'angle_max': math.pi,
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.05,
                'range_max': 5.0,
                'use_inf': False,
                'inf_epsilon': 4.0,
                'qos': "best_effort",
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'scan_delay': 0.01,
            }],
            name='utlidar_to_scan'
        ),
        fake_scan_move
    ])

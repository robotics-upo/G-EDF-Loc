import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_path',
            default_value='/home/upo/Datasets/college_dataset/map/new_college_v1.bin',
            description='Path to the .bin map file'
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.3',
            description='Resolution for sampling the map'
        ),
        DeclareLaunchArgument(
            'iso_threshold',
            default_value='0.001',
            description='Threshold for considering a voxel occupied'
        ),
        
        Node(
            package='g_edf_loc',
            executable='map_publisher_node',
            name='map_publisher_node',
            output='screen',
            parameters=[{
                'map_path': LaunchConfiguration('map_path'),
                'resolution': LaunchConfiguration('resolution'),
                'iso_threshold': LaunchConfiguration('iso_threshold'),
                'map_frame': 'map' 
            }]
        )
    ])

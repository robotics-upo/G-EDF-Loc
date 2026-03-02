from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('g_edf_loc')
    config_file = os.path.join(pkg_share, 'config', 'lidar_localization.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'localization.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz'
        ),
        
        DeclareLaunchArgument(
            'mode',
            default_value='default',
            description='Mode: default, no_imu'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='x36d',
            arguments=[
                "-0.06","0.0","-0.16",
                "-0.0007904608601149109","-0.003950257535560073","-0.0007041813888545112","0.9999916373478345",
                'base_link',
                'x36d'
            ],
            output='screen' 
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='Body_T_Xt32',
            arguments=[
                "0.0", "0.0", "0.0",
                "0.0", "0.0", "0.7071067811865475", "0.7071067811865475",
                "base_link",
                "PandarXT-32"
            ],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_os_sensor',  
            arguments=[
                "0.001", "0.0", "0.091",           
                "0.0", "0.0", "0.0", "1.0",        
                "base_link",                       
                "os_sensor"                        
            ],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_os_imu',    
            arguments=[
                "0.015", "-0.012", "0.076",        
                "0.0", "0.0", "0.0", "1.0",        
                "base_link",                       
                "os_imu"                           
            ],
            output='screen'
        ),
        # Default Node
        Node(
            package='g_edf_loc',
            executable='lidar_localization_node',
            name='lidar_localization_node',
            output='screen',
            parameters=[config_file],
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'default'"]))
        ),

        # No IMU Node
        Node(
            package='g_edf_loc',
            executable='lidar_localization_no_imu_node',
            name='lidar_localization_node',
            output='screen',
            parameters=[config_file],
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'no_imu'"]))
        )



    ])

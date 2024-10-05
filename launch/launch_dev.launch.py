import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = 'lizardbot1'
    pkg_path = os.path.join(get_package_share_directory(package_name))
    rviz_config_path = os.path.join(pkg_path,'config','drive_config.rviz')

    joystick_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_path,'launch','joystick.launch.py')])
    )

    # Create rviz2 node		
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(name='rvizconfig',
                              default_value=rviz_config_path,
                              description='Absolute path to rviz config file'),

        joystick_launch_file,
        rviz_node
        # [TODO] slam_toolbox: cpy original launch file to lizardbot1 package, and include launch here
        # [TODO] nav2: cpy original launch file to lizardbot1 package, and include launch here
    ])
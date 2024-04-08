import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
	# Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
	# !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

	package_name='lizardbot1'

	# Include the launch file
	rsp = IncludeLaunchDescription(
			PythonLaunchDescriptionSource([os.path.join(
				get_package_share_directory(package_name),'launch','rsp.launch.py'
			)]), launch_arguments={'use_sim_time': 'True'}.items()
	)

	# Include the Gazebo launch file, provided by the gazebo_ros package
	gazebo = IncludeLaunchDescription(
				PythonLaunchDescriptionSource([os.path.join(
					get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
			)

	# Run the spawner node from the gazebo_ros package. Create a ROS2 to spawn the robot.
	spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
						arguments=['-topic', 'robot_description',
								'-entity', 'lizardbot1'],
						output='screen')

	# Launch
	return LaunchDescription([
		rsp,
		gazebo,
		spawn_entity,
	])
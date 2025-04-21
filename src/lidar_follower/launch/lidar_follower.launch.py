import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
	pkg_path = get_package_prefix('lidar_follower')
	rviz_config_path = os.path.join(pkg_path, 'share/lidar_follower/rviz', 'lidar_follower.rviz')
    
	return LaunchDescription([
		Node(
			package='lidar_follower',
			executable='lidar_filter',
			name='lidar_filter'
		),
		Node(
			package='lidar_follower',
			executable='lidar_follower',
			name='lidar_follower'
		),
		Node(
			package='rviz2',
			executable='rviz2',
			name='rviz2',
			output='screen',
			arguments=['-d', rviz_config_path]
		)
	])

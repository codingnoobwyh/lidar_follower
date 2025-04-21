from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
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
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '/home/robot/Desktop/lidar_bag/lidar_bag.db3'],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        )
    ])

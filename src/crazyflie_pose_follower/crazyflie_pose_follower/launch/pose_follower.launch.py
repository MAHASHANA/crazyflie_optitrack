from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='crazyflie_pose_follower',
            executable='pose_follower_node',
            name='pose_follower_node',
            output='screen'
        )
    ])
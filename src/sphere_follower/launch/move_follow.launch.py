import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sphere_follower',  
            executable='move_sphere',
            name='sphere_mover_node',
            output='screen',
        ),
        Node(
            package='sphere_follower',  
            executable='follow_sphere',
            name='pioneer_follower_node',
            output='screen',
        ),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='multi_robot_challenge_23',
            executable='robot_controller',
            name='tb3_1_controller',
            namespace='tb3_1',
            output='screen',
            arguments=['tb3_1']
        ),
        Node(
            package='multi_robot_challenge_23',
            executable='robot_controller',
            name='tb3_2_controller',
            namespace='tb3_2',
            output='screen',
            arguments=['tb3_2']
        ),

        Node(
            package='multi_robot_challenge_23',
            executable='leader',
            name='leader',
            output='screen'
        ),
    ])

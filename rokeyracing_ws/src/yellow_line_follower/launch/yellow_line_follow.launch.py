from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yellow_line_follower',
            executable='detect_yellow_line',
            name='detect_yellow_line',
            output='screen',
            remappings=[
                ('/camera/image_projected', '/camera/image_projected'),
                ('/detect/yellow_line_mask', '/detect/yellow_line_mask')
            ]
        ),
        Node(
            package='yellow_line_follower',
            executable='yellow_line_center_pub',
            name='yellow_line_center_pub',
            output='screen',
            remappings=[
                ('/detect/yellow_line_mask', '/detect/yellow_line_mask'),
                ('/detect/lane', '/detect/lane')
            ]
        )
    ])

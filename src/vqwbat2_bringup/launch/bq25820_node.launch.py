from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bq25820_node',
            executable='bq25820_node',
            name='bq25820_node',
            output='screen',
            parameters=[
                # Add your parameters here if needed
            ]
        )
    ])
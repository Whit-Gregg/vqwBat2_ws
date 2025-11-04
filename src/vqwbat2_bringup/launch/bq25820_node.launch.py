# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='bq25820_node',
#             executable='bq25820_node',
#             name='bq25820_node',
#             output='screen',
#             parameters=[
#                 # Add your parameters here if needed
#             ]
#         )
#     ])

from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node
from launch.actions import ExecuteProcess
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ld = LaunchDescription()


    BQ25820_node = Node(
        package="bq25820_node",
        executable="bq25820_node",
        ##plugin="bq25820_node::BQ25820Node",
        name="bq25820_node",
        output="both",
        parameters=[
            # Add your parameters here if needed
        ]
    )
    
    ld.add_action(BQ25820_node)

    return ld


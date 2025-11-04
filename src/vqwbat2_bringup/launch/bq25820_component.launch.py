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

    container = Node(
        name="bq25820_container",
        package="rclcpp_components",
        executable="component_container",
        #output="both",
    )
    ld.add_action(container)

    BQ25820_node = ComposableNode(
        package="bq25820_node",
        ##executable="bq25820_node",
        plugin="bq25820_node::BQ25820Node",
        name="bq25820_node",
        #output="screen",
        parameters=[
            # Add your parameters here if needed
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    
    loader = LoadComposableNodes(
        target_container="bq25820_container",
        composable_node_descriptions=[
            BQ25820_node,
        ],
    )
    ld.add_action(loader)

    lifecycle_configure = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "--spin-time", "5", "bq25820_node", "configure"],
        output="screen",
    )
    ld.add_action(lifecycle_configure)

    lifecycle_activate = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "--spin-time", "5", "bq25820_node", "activate"],
        output="screen",
    )
    ld.add_action(lifecycle_activate)

    return ld


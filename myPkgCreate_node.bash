#!/bin/bash


ros2 pkg create --license MIT --build-type ament_cmake --destination-directory src --maintainer-email whitgregg@hotmail.com --maintainer-name "Whit Gregg"  --dependencies rclcpp --node-name $2 $1

# ros2 pkg create --license MIT --build-type ament_cmake --destination-directory src \
#     --maintainer-email whitgregg@hotmail.com --maintainer-name "Whit Gregg"  \
#     --dependencies rclcpp std_msgs rclcpp_lifecycle rclcpp_components \
#     --node-name LifecycleManager vqw_lifecycle_manager

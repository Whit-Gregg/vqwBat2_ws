#pragma once

#include <rclcpp/rclcpp.hpp>

// // // #include </opt/ros/kilted/include/rclcpp/rclcpp/rclcpp.hpp>


#define BUG_INFO(txt) RCLCPP_INFO(rclcpp::get_logger("BQ25820"), txt);

// #define BUG_INFO(txt) printf("%s",txt);

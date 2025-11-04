cmake_minimum_required(VERSION 3.20)
project(bq25820_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bq25820 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)


add_executable(bq25820_node src/bq25820_node.cpp)
## add_library(bq25820_component SHARED src/bq25820_node.cpp)

target_compile_features(bq25820_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(bq25820_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(bq25820_node PUBLIC
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  bq25820::bq25820
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
)

# # # rclcpp_components_register_node(bq25820_node
# # #   PLUGIN "bq25820_node::BQ25820Node"
# # #   EXECUTABLE bq25820_node
# # # )

# # # ament_export_libraries(bq25820_component)

#ament_export_targets(export_bq25820_component)
# ament_export_targets(bq25820_component
#   EXPORT export_bq25820_component
#   )

# install(TARGETS bq25820_node
#         EXPORT export_bq25820_node
#         ARCHIVE DESTINATION lib
#         LIBRARY DESTINATION lib
#         RUNTIME DESTINATION bin
# )

install(TARGETS bq25820_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)

ament_package()

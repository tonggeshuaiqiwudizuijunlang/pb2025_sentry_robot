cmake_minimum_required(VERSION 3.8)
project(simple_sentry_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pb_rm_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

add_library(simple_sentry_control_node SHARED src/simple_sentry_control_node.cpp)

add_executable(waypoint_manager src/waypoint_manager.cpp)
ament_target_dependencies(waypoint_manager rclcpp rclcpp_action nav2_msgs geometry_msgs std_srvs)

target_include_directories(simple_sentry_control_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(simple_sentry_control_node PUBLIC c_std_99 cxx_std_17)  # Require C++17
ament_target_dependencies(
  simple_sentry_control_node
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "nav2_msgs"
  "pb_rm_interfaces"
  "std_srvs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "example_interfaces"
)
rclcpp_components_register_node(simple_sentry_control_node PLUGIN "simple_sentry_control::SimpleSentryControlNode" EXECUTABLE simple_sentry_control_node_exec)

add_library(simple_sentry_control_two_point_node SHARED src/simple_sentry_control_two_point_node.cpp)
target_include_directories(simple_sentry_control_two_point_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(simple_sentry_control_two_point_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  simple_sentry_control_two_point_node
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "nav2_msgs"
  "pb_rm_interfaces"
  "tf2_geometry_msgs"
  "example_interfaces"
)
rclcpp_components_register_node(simple_sentry_control_two_point_node PLUGIN "simple_sentry_control::SimpleSentryControlTwoPointNode" EXECUTABLE simple_sentry_control_two_point_node_exec)

install(TARGETS
  simple_sentry_control_node
  simple_sentry_control_node_exec
  simple_sentry_control_two_point_node
  simple_sentry_control_two_point_node_exec
  waypoint_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

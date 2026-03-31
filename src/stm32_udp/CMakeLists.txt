cmake_minimum_required(VERSION 3.20)
project(stm32_udp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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

add_executable(stm32_udp_odom src/stm32_udp_odom.cpp)
target_link_libraries(
      stm32_udp_odom PUBLIC
      ${geometry_msgs_TARGETS}
      ${nav_msgs_TARGETS}
      rclcpp::rclcpp
      tf2::tf2
      tf2_geometry_msgs::tf2_geometry_msgs
      tf2_ros::static_transform_broadcaster_node
      tf2_ros::tf2_ros
)

#ament_target_dependencies(stm32_udp_odom rclcpp nav_msgs geometry_msgs tf2 tf2_ros)
install(TARGETS stm32_udp_odom DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  rviz
  params
  DESTINATION share/${PROJECT_NAME})

ament_package()

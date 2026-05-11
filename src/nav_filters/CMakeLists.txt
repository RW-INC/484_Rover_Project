cmake_minimum_required(VERSION 3.20)
project(nav_filters)
list(APPEND CMAKE_PREFIX_PATH "/usr/share/eigen3/cmake")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(nav_node src/main.cpp)
target_link_libraries(nav_node PUBLIC
      ${geometry_msgs_TARGETS}
      ${sensor_msgs_TARGETS}
      ${std_msgs_TARGETS}
      ${nav_msgs_TARGETS}
      ${message_filters_TARGETS}
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs_library
    )



add_executable(auto_pub_node src/auto_pub.cpp)
target_link_libraries(auto_pub_node PUBLIC
      ${geometry_msgs_TARGETS}
      ${sensor_msgs_TARGETS}
      ${std_msgs_TARGETS}
      ${nav_msgs_TARGETS}
      ${message_filters_TARGETS}
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs_library
    )

install(TARGETS
  auto_pub_node nav_node
  DESTINATION lib/${PROJECT_NAME}
)


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

ament_package()

cmake_minimum_required(VERSION 3.20)
project(motor_commander)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})


add_executable(smc_node src/smc_main.cpp)
target_include_directories(smc_node PRIVATE include)
target_link_libraries(smc_node PUBLIC
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
)

add_executable(motor_commander_node src/mc_main.cpp)
target_include_directories(motor_commander_node PRIVATE include)
target_link_libraries(motor_commander_node PUBLIC
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
)

install(TARGETS smc_node motor_commander_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
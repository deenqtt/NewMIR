cmake_minimum_required(VERSION 3.5)
project(newmir)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

install(DIRECTORY launch maps params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

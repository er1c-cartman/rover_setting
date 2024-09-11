cmake_minimum_required(VERSION 3.5)
project(four_wheel_robot_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)

include_directories(
  include
  ${dynamixel_sdk_INCLUDE_DIRS}
)

add_executable(four_wheel_kinematics src/four_wheel_kinematics.cpp)
ament_target_dependencies(four_wheel_kinematics
  rclcpp
  geometry_msgs
  dynamixel_sdk
  dynamixel_sdk_custom_interfaces
)

install(TARGETS
  four_wheel_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()


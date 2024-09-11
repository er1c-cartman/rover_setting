cmake_minimum_required(VERSION 3.8)
project(rover_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(xacro REQUIRED)  # Add xacro as a required package

# Build the custom node
add_executable(fake_odom_publisher src/fake_odom_publisher.cpp)
ament_target_dependencies(fake_odom_publisher rclcpp nav_msgs geometry_msgs tf2_ros tf2 tf2_geometry_msgs)

# Install the executable
install(TARGETS
  fake_odom_publisher
  DESTINATION lib/${PROJECT_NAME})

# Install other resources (URDF, Xacro, meshes, launch, and RViz files)
install(DIRECTORY urdf meshes launch rviz
    DESTINATION share/${PROJECT_NAME}
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


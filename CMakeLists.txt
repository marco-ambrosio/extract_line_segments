cmake_minimum_required(VERSION 3.5)
project(extract_line_segments)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Define the executable and link dependencies
add_executable(extract_line_segments_node src/extract_line_segments_node.cpp)

ament_target_dependencies(extract_line_segments_node
  rclcpp
  sensor_msgs
  visualization_msgs
  laser_geometry
  pcl_conversions
  geometry_msgs
)

target_link_libraries(extract_line_segments_node ${PCL_LIBRARIES})

# Install the executable
install(TARGETS
  extract_line_segments_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files, if any
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Ament package
ament_package()

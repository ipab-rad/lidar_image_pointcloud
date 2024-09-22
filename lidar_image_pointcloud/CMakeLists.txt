cmake_minimum_required(VERSION 3.8)
project(lidar_image_pointcloud)

# Set C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED)
if(OpenCV_VERSION VERSION_LESS "3.2.0")
  message(FATAL "Minimum OpenCV version is 3.2.0 (found version ${OpenCV_VERSION})")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)

include_directories(include)

add_library(lidar_image_pointcloud_component SHARED src/lidar_image_pointcloud.cpp)

ament_target_dependencies(lidar_image_pointcloud_component
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  cv_bridge
  image_geometry
  image_transport
)



target_link_libraries(lidar_image_pointcloud_component
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(
  lidar_image_pointcloud_component
    PLUGIN "sensing::LidarImagePointcloud"
    EXECUTABLE lidar_image_pointcloud
)


ament_export_targets(export_lidar_image_pointcloud_component)
install(TARGETS lidar_image_pointcloud_component
        EXPORT export_lidar_image_pointcloud_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

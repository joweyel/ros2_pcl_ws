cmake_minimum_required(VERSION 3.8)
project(process_pcl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(VTK 9 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS} ${VTK_LIBRARY_DIRS})

add_executable(data_processing src/data_processing2.cpp src/pcl_utils.cpp)

target_link_libraries(data_processing
  ${rclcpp_LIBRARIES}
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(data_processing
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  VTK
  image_transport
  OpenCV
  cv_bridge
  visualization_msgs
)

# Install the executable 
install(
  TARGETS data_processing 
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()

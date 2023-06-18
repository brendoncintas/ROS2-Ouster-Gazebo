cmake_minimum_required(VERSION 3.5)
project(ouster_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(gazebo_ros_ouster_laser SHARED src/GazeboRosOusterLaser.cpp)
ament_target_dependencies(gazebo_ros_ouster_laser
  rclcpp
  sensor_msgs
  tf2
  gazebo_ros_pkgs
)
target_link_libraries(gazebo_ros_ouster_laser
  ${ament_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  RayPlugin
)

add_library(gazebo_ros_ouster_gpu_laser SHARED src/GazeboRosOusterLaser.cpp)
ament_target_dependencies(gazebo_ros_ouster_gpu_laser
  rclcpp
  sensor_msgs
  tf2
  gazebo_ros_pkgs
)
target_link_libraries(gazebo_ros_ouster_gpu_laser
  ${ament_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  GpuRayPlugin
)
target_compile_definitions(gazebo_ros_ouster_gpu_laser PRIVATE GAZEBO_GPU_RAY=1)

install(TARGETS gazebo_ros_ouster_laser gazebo_ros_ouster_gpu_laser
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()

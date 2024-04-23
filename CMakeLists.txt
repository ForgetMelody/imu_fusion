cmake_minimum_required(VERSION 3.0.2)
project(imu_preintegration)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
)

catkin_package(
)

include_directories(/usr/local/include/eigen3)

include_directories(
# include
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  src
)

add_definitions(${EIGEN_DEFINITIONS})

## Declare a C++ executable
add_executable(imu_node 
    src/imu_process.cpp 
    src/ros_interface/ros_interface.cpp
    src/kalman_filter/kf.cpp
)

target_link_libraries(imu_node ${catkin_LIBRARIES})


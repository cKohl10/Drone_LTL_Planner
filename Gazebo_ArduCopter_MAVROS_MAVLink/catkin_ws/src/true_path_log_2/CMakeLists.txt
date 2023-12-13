cmake_minimum_required(VERSION 3.0.2)
project(true_path_log_2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES true_path_log
  CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(true_path_log_2 src/new_path.cpp)
target_link_libraries(true_path_log_2 ${catkin_LIBRARIES})
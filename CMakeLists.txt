cmake_minimum_required(VERSION 3.0.2)
project(offboard)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  roslaunch
  rospy
  std_msgs
  nav_msgs
  mav_trajectory_generation 
  mav_trajectory_generation_ros
  message_generation
)

add_message_files(
  FILES
  FlatTarget.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES offboard
   CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp rospy std_msgs nav_msgs mav_trajectory_generation mav_trajectory_generation_ros message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

roslaunch_add_file_check(launch)

add_library(offboard_lib
  src/offboard_lib.cpp
)
target_link_libraries(offboard_lib
  ${catkin_LIBRARIES}
)

add_library(ivsr_log
  src/ivsr_log.cpp
)
target_link_libraries(ivsr_log
  ${catkin_LIBRARIES}
)

add_executable(offboard_node src/offboard_node.cpp)
target_link_libraries(offboard_node
  offboard_lib
  ivsr_log
)

add_executable(setmode_offb src/setmode_offb.cpp)
target_link_libraries(setmode_offb
  ${catkin_LIBRARIES}
)
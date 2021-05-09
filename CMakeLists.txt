cmake_minimum_required(VERSION 3.0.2)
project(offboard)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  rospy
  std_msgs
  roslaunch
  mav_trajectory_generation
  mav_trajectory_generation_ros
  mav_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES offboard
   CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp rospy std_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

roslaunch_add_file_check(launch)

add_library(
  offboard_lib
  src/offboard_lib.cpp
)

target_link_libraries(offboard_lib
  ${catkin_LIBRARIES}
)

add_executable(offboard src/offboard_node.cpp)
add_executable(hover src/hover_node.cpp)
add_executable(setmode_offb src/setmode_offb.cpp)
add_executable(trigger_pub src/trigger_pub.cpp)

target_link_libraries(offboard
  offboard_lib
)
target_link_libraries(hover
  ${catkin_LIBRARIES}
)
target_link_libraries(setmode_offb
  offboard_lib
)
target_link_libraries(trigger_pub
  ${catkin_LIBRARIES}
)
# offboard package

## contain
- *include/offboard/offboard.h*
- *src/hovering_node.cpp*: keep drone hovering on input target position
- *src/offboard_node.cpp*: keep drone flying follow input waypoints
- *package.xml*: ros manifests
- *CMakeLists.txt*: CMake

## required
- **ros**: Melodic (on Ubuntu 18.04)
- **catkin workspace**: `catkin_ws`
- **mavros**: at `catkin_ws/src/mavros`
- **mavlink**: at `catkin_ws/src/mavlink`

## usage
###### hovering node
- *connect jetson to pixhawk*         : `roslaunch mavros px4.launch`
- *run hovering_node*                 : `rosrun offboard hovering`
- **check current state and position on screen; then input target (x,y,z) position**
- **on remote controller** switch to ARM, then switch flight mode to OFFBOARD

###### offboard node
- *connect jetson to pixhawk*         : `roslaunch mavros px4.launch`
- *run offboard_node*                 : `rosrun offboard offboard`
- **check current state and position on screen; then input number of target and target[i] (x_i,y_i,z_i) positions**
- **on remote controller** switch to ARM, then switch flight mode to OFFBOARD

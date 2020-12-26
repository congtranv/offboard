# offboard package

## contain
- *include/offboard/offboard.h*  : header
- *include/offboard/conversion.h*: header
- *src/setpoint_node.cpp*        : offboard position control: local anf global setpoint
- *src/setmode_offb.cpp*         : set OFFBOARD mode and ARM vehicle in simulation
- *launch/offboard.launch*       : ros launch file
- *package.xml*                  : ros manifests
- *CMakeLists.txt*               : CMakeLists

## required
- **ros**             : Melodic (on Ubuntu 18.04)
- **catkin workspace**: `catkin_ws`
- **mavros**          : [here](https://dev.px4.io/master/en/ros/mavros_installation.html)

- **copy `offboard` directory to `catkin_ws/src/` and build**
- **PX4 Firmware**    : tested on v1.10.1

## usage
#### bash script is [here](https://github.com/congtranv/bash)

###### practice
- *connect jetson to pixhawk*         : `roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600`  
- *run setpoint_node*                 : `rosrun offboard setpoint` or `roslaunch offboard offboard.launch`
- **check status on screen**

  **following the guide on screen**
  
- **on remote controller** switch ARM, then switch flight mode to OFFBOARD

###### simulation
- *run simulation*                    : `roslaunch px4 mavros_posix_sitl.launch`
- *run setpoint_node*                 : `rosrun offboard setpoint` or `roslaunch offboard offboard.launch`
- **check status on screen**

  **following the guide on screen**
    
- **RC simulation** switch to ARM, then switch flight mode to OFFBOARD

 `rosrun offboard setmode_offb`
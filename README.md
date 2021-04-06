# ivsr offboard package

## contain
- *include/offboard/offboard.h* : header offboard

- *src/hover_node.cpp*      : keep drone hovering on input z height
- *src/offboard_node.cpp*   : initial ros node
- *src/offboard_lib.cpp*    : library for offboard node
- *src/setmode_offb.cpp*    : set OFFBOARD mode and ARM vehicle in simulation

- *config/waypoints.yaml*   : prepared params to load into offboard node
- *package.xml*             : ros manifests
- *CMakeLists.txt*          : CMakeLists

## required
- **ros**             : tested on Melodic (Ubuntu 18.04)
- **PX4**             : tested on v10.0.1 
- **catkin workspace**: `catkin_ws`
- **mavros**          : binary installation [here](https://docs.px4.io/master/en/ros/mavros_installation.html#binary-installation-debian-ubuntu)

- **git clone `offboard` to `catkin_ws/src/` and build `catkin build`**

## usage

### 1. CONNECT COMPANION PC TO PIXHAWK or RUN SIMULATION
#### 1.1 Practice on jetson
- **connect jetson to pixhawk** (*replace fcu_url with your own config*)

  ```
  roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600
  ```
#### 1.2 run simulation
- **run px4 simulation with mavros connected**

  ```
  roslaunch px4 mavros_posix_sitl.launch
  ```

### 2. RUN OFFBOARD
#### 2.1 hovering node
- *run hover_node*: 

  ```
  rosrun offboard hover
  ```
- **check current position on screen**

  **input target height for hovering (in meter): z**
  
- **on remote controller** switch to ARM, then switch flight mode to OFFBOARD

  on simualation: 
  
  ```
  rosrun offboard setmode_offb
  ```
### or:
#### 2.2 offboard node
- *run offboard_node*: 

  ```
  rosrun offboard offboard
  ```
- **manual input or load input from waypoints.yaml config file**
  
- **on remote controller** switch to ARM, then switch flight mode to OFFBOARD

  on simualation: 
  
  ```
  rosrun offboard setmode_offb
  ```
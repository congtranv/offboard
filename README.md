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

### 1. HOVERING node
#### 1.1 CONNECT TO PIXHAWK
##### 1.1.1 Practice on jetson
- **connect jetson to pixhawk** (*replace fcu_url with your own config*)

  ```
  roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600
  ```
##### 1.1.2 run simulation
- **run px4 simulation with mavros connected**

  ```
  roslaunch px4 mavros_posix_sitl.launch
  ```
#### 1.2 run *hover_node*: 

  ```
  rosrun offboard hover
  ```
- **check current position on screen**

  **input target height for hovering (in meter): z**

#### 1.3 ARM and switch mode 
- **on remote controller** switch to ARM, then switch flight mode to OFFBOARD

  on simualation: 
  
  ```
  rosrun offboard setmode_offb
  ```
### 2. OFFBOARD node
#### 2.1 CONNECT TO PIXHAWK
##### 2.1.1 Practice on jetson
- **connect jetson to pixhawk** (*replace fcu_url with your own config*)

  ```
  roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600
  ```
  ***
  in this terminal, run  `param set COM_DISARM_LAND -1`  to disable auto disarm when land of pixhawk:
  ```
  pxh> param set COM_DISARM_LAND -1
  ```
  when practice done, set COM_DISARM_LAND to 2 for enable auto disarm after land 2 seconds:
  ```
  pxh> param set COM_DISARM_LAND 2
  ```
  ***
##### 2.1.2 run simulation
- **run px4 simulation with mavros connected**

  ```
  roslaunch px4 mavros_posix_sitl.launch
  ```
#### 2.2 run *offboard_node*: 

  ```
  rosrun offboard offboard
  ```
- **manual input or load input from waypoints.yaml config file**
  
#### 2.3 ARM and switch mode
- **on remote controller** switch to ARM, then switch flight mode to OFFBOARD

- on simualation: 
  
  ```
  rosrun offboard setmode_offb
  ```
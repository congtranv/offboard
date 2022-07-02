# IVSR OFFBOARD package

***
## !!! WARNING

__*OFFBOARD* control is dangerous.__

**If you are operating on a real vehicle be sure to have a way of gaining back manual control in case something goes wrong.**
***

## Contain
- *include/offboard/offboard.h*: header offboard

- *src/offboard_node.cpp*: offboard node source code
- *src/offboard_lib.cpp*: library for offboard node
- *src/setmode_offb.cpp*: set OFFBOARD mode and ARM vehicle in simulation
- *launch/offboard.launch*: launch file, include parameter

## Required

### Environments
- **ROS**: tested on [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) (Ubuntu 18.04)
- **PX4 Firmware**: tested on v10.0.1 - setup [here](https://github.com/congtranv/px4_install)
- **Catkin workspace**: `catkin_ws`
  ```
  ## create a workspace if you've not had one
  mkdir -p [path/to/ws]/catkin_ws/src
  cd [path/to/ws]/catkin_ws
  ```
  ```
  catkin init
  catkin config --extend /opt/ros/melodic
  catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
  catkin config --merge-devel
  catkin build
  ```
- **MAVROS**: binary installation - setup [here](https://docs.px4.io/master/en/ros/mavros_installation.html#binary-installation-debian-ubuntu)

### [ethz-asl/mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation) 

```
cd [path/to/ws]/catkin_ws/src
```
```
git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/ethz-asl/eigen_catkin.git
git clone https://github.com/ethz-asl/eigen_checks.git
git clone https://github.com/ethz-asl/nlopt.git
git clone https://github.com/ethz-asl/glog_catkin.git
git clone https://github.com/ethz-asl/mav_comm.git
git clone https://github.com/ethz-asl/yaml_cpp_catkin.git
git clone https://github.com/ethz-asl/mav_trajectory_generation.git
```
```
cd [path/to/ws]/catkin_ws
```
```
catkin build
```
### [congtranv/offboard](https://github.com/congtranv/offboard)
```
cd [path/to/ws]/catkin_ws/src
```
```
git clone https://github.com/congtranv/offboard.git
```
```
cd [path/to/ws]/catkin_ws
```
```
catkin build offboard
```

## Usage
***
**IVSR Tutorial: Read from  [onedrive](https://husteduvn-my.sharepoint.com/:w:/g/personal/quang_nguyenanh_hust_edu_vn/EdFUubKnGGFOuTbTw5xGu3IB2g8LsIFqIswVKiPCkyMmTw?e=uTuyDv)**
***
***
### <span style="color:green">*Before run OFFBOARD node, check and modify (if need) the value of parameters in* **launch/offboard.launch**
***
### There 2 main functions:
- <span style="color:violet">HOVERING</span>: drone hover at `z` meters (input from keyboard) in `hover_time` seconds (change in launch/offboard.launch)
- <span style="color:violet">MISSION</span>: fly with the local/GPS setpoints that prepared in launch/offboard.launch or input from keyboard
### <span style="color:green">Refer the [test_case.md](test_case.md) for all detail use cases of OFFBOARD node

### <span style="color:yellow">1. Simulation (SITL)

#### <span style="color:cyan">1.1 Run PX4 simulation
```
roslaunch px4 mavros_posix_sitl.launch
```
***
  If run OFFBOARD Mission with Delivery mode *(landing at each setpoint)* -
(`roslaunch offboard offboard.launch simulation:=true delivery:=true z_delivery:=0.0`)

  In this terminal (Run PX4 simulation), run  `param set COM_DISARM_LAND -1`  to disable auto disarm when land of pixhawk:
  ```
  pxh> param set COM_DISARM_LAND -1
  ```
  When done, set `COM_DISARM_LAND` to `2` for enable auto disarm after land 2 seconds:
  ```
  pxh> param set COM_DISARM_LAND 2
  ```
***
#### <span style="color:cyan">1.2 Run OFFBOARD node
```
roslaunch offboard offboard.launch simulation:=true
```

If forgot parameter `simulation`, can use setmode node (in another terminal)
```
rosrun offboard setmode_offb
```
*(maybe have to run twice for ARM and Switch OFFBOARD mode)*
### <span style="color:yellow">2. Practice in test field

***
  Before run OFFBOARD Mission with Delivery mode *(landing at each setpoint)* (`roslaunch offboard offboard.launch delivery:=true z_delivery:=0.0`), use [QGround Control](https://github.com/congtranv/px4-param/blob/main/QGroundControl.AppImage) to set parameter `COM_DISARM_LAND` to `-1`*(disable)* for disable auto disarm when drone landed. 
  
  **Use Remote controller to DISARM when mission completed**

  When practice done, **HAVE TO** set `COM_DISARM_LAND` to `2` for enable auto DISARM when drone landed after 2 seconds.
***

##### <span style="color:green">***(can use for HITL simulation)***

#### <span style="color:cyan">2.1 Connect Companion PC to Pixhawk 4 
```
roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600
```
#### <span style="color:cyan">2.2 Run OFFBOARD node
```
roslaunch offboard offboard.launch
```
#### <span style="color:cyan">2.3 ARM and switch to OFFBOARD mode
Use Remote controller to ARM and switch flight mode to OFFBOARD


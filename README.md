# IVSR OFFBOARD package

***
## <span style="color:red">!!! WARNING

<span style="color:yellow">__*OFFBOARD* control is dangerous.__

<span style="color:yellow">**If you are operating on a real vehicle be sure to have a way of gaining back manual control in case something goes wrong.**
***

## Contain
- <span style="color:orange">*include/offboard/offboard.h*</span> : header offboard

- <span style="color:orange">*src/offboard_node.cpp*</span>   : offboard node source code
- <span style="color:orange">*src/offboard_lib.cpp*</span>    : library for offboard node
- <span style="color:orange">*src/setmode_offb.cpp*</span>    : set OFFBOARD mode and ARM vehicle in simulation
- <span style="color:orange">*launch/offboard.launch*</span>  : launch file, include parameter

## Required
- <span style="color:orange">**ROS**</span>             : tested on ROS Melodic (Ubuntu 18.04)
- <span style="color:orange">**PX4 Firmware**</span>    : tested on v10.0.1 - setup [here](https://github.com/congtranv/px4_install)
- <span style="color:orange">**Catkin workspace**</span>: `catkin_ws`
  ```
  ## create a workspace if you've not had one
  mkdir -p [path/to/ws]/catkin_ws/src
  cd [path/to/ws]/catkin_ws
  catkin_init_workspace
  rosdep install --from-paths src --ignore-src -y 
  catkin build
  ```
- <span style="color:orange">**MAVROS**</span>          : binary installation - setup [here](https://docs.px4.io/master/en/ros/mavros_installation.html#binary-installation-debian-ubuntu)

- <span style="color:orange">**OFFBOARD**</span>
  ```
  cd [path/to/ws]/catkin_ws/src
  git clone https://github.com/congtranv/offboard.git
  cd [path/to/ws]/catkin_ws
  catkin build offboard
  ```

## Usage
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
  In this terminal, to run OFFBOARD Mission with Delivery mode *(landing at each setpoint)*

  run  `param set COM_DISARM_LAND -1`  to disable auto disarm when land of pixhawk:
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
  Before run OFFBOARD Mission with Delivery mode *(landing at each setpoint)*, use [QGround Control](https://github.com/congtranv/px4-param/blob/main/QGroundControl.AppImage) to set parameter `COM_DISARM_LAND` to `-1`*(disable)* for disable auto disarm when drone landed. 
  
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
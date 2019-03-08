# Ardupilot Gazebo ROS Package

## Requirements
- Ubuntu (16.04 LTS), able to run full 3D graphics.
- Gazebo (version 7.x or 8.x)
- ROS Kinetic
- gazebo_ros_pkgs
- mavros

## Installation List
### Ubuntu 16.04 LTS
[Installation](https://www.ubuntu.com/download/alternative-downloads)
### ROS (kinetic)  
[Installation](http://wiki.ros.org/kinetic/Installation/Ubuntu)  
### SITL Simulator  
[Document](http://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)  
[Source from RMackay9](https://github.com/rmackay9/rmackay9-ardupilot)  
[Source from Ardupilot](https://github.com/ArduPilot/ardupilot)  
[Installation Instruction](http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)
### MAVProxy  
We are using MavProxy 1.5.7  
So please run `pip install mavproxy==1.5.7` to install MavProxy  
[Document](https://ardupilot.github.io/MAVProxy/html/getting_started/download_and_installation.html)
### MAVROS
[Instruction](https://dev.px4.io/en/ros/mavros_installation.html)
### Gazebo 
[Instruction](https://dev.px4.io/en/setup/dev_env_linux.html#gazebo-with-ros)
### Ardupilot Gazebo
[Source from Vince Kurtz](https://github.com/vincekurtz/ardupilot_gazebo)  
[Instruction](http://ardupilot.org/dev/docs/using-gazebo-simulator-with-sitl.html)  


## Quadcopter Control Document
To simulate no gps availability to mimic our project conditions, set the SIM_GPS_DISABLE parameter to 1. (Can be done using param set SIM_GPS_DISABLE 1 in the MAVProxy shell, or probably using QGroundControl)
Disable all pre-arm checks in QGC
> param set ATC_RAT_RLL_D 0.000100
Set COMPASS_USE1, 2, 3 = 0
FS_EKF_THRESH = 1
Found these on the forums ( AHRS_EKF_TYPE 2 EKF2_ENABLE 1 EKF3_ENABLE 0 GPS_TYPE 0 EK2_GPS_TYPE 3 COMPASS_USE 0 VISO_TYPE 0)

## Examples

### Basic IRIS simulation

Start the ardupilot simulation
```
sim_vehicle.py -v ArduCopter -f gazebo-iris  -m --mav10 -I0
```

Launch the gazebo environment
```
roslaunch ardupilot_gazebo iris_world.launch
```

After confirming a GPS fix (this may take a minute), control the the model through the SITL prompt. You can also connect apmplanner2 or your favorite GCS to the simulated model.
```
> arm throttle
> mode GUIDED
> takeoff 3
> mode LAND
```

### Simulate an optitrack motion capture system

Start the simulation with the proper parameters
```
sim_vehicle.py -v ArduCopter -f gazebo-iris -m --mav10 -I0 --add-param-file="$(rospack find ardupilot_gazebo)/sitl_parameters/optitrack.parm"
```

Launch the gazebo environment with a simulated mocap system (based on the actual pose from Gazebo)
```
roslaunch ardupilot_gazebo iris_world_optitrack.launch
```

This will give a much more accurate position than the simulated GPS. 

### Multi-agent Simulation

Start two instances of SITL in different terminals (note that I0 and I1 are different)
```
sim_vehicle.py -v ArduCopter -f gazebo-iris  -m --mav10 -I0"
```
```
sim_vehicle.py -v ArduCopter -f gazebo-iris  -m --mav10 -I1"
```

Launch the multi-agent gazebo file
```
roslaunch ardupilot_gazebo multi_agent_iris.launch
```

After console shows using GPS, run demo duo control  
```
rosrun ardupilot_gazebo demo_duo_control.py
```

## Troubleshooting

### mavros_node error

Sample error messages:  
`/opt/ros/kinetic/lib/mavros/mavros_node: symbol lookup error: /opt/ros/kinetic/lib//libmavros_plugins.so: undefined symbol: _ZN7mavconn16MAVConnInterface24send_message_ignore_dropERKN7mavlink7MessageEh`  
```
================================================================================REQUIRED process [uav2/mavros-5] has died!
process has died [pid 4257, exit code -6, cmd /opt/ros/kinetic/lib/mavros/mavros_node __name:=mavros __log:=/home/dronear/.ros/log/07d26dca-413a-11e9-9c9c-3497f69d0ed2/uav2-mavros-5.log].
log file: /home/dronear/.ros/log/07d26dca-413a-11e9-9c9c-3497f69d0ed2/uav2-mavros-5*.log
Initiating shutdown!
================================================================================
```

Please upgrade modules
```bash
sudo apt update
sudo apt upgrade
```

### Cannot compile
If mavlink and mavros are in src folder, try `catkin_make_isolated`  
If cannot compile ardupilot_gazebo, please check if gazebo version is 7.14 or 8.0 (only two versions of Gazebo work fine)  

### Cannot find ardupilot_gazebo use rosrun or roslaunch
Try `source ./catkin_ws/devel/setup.bash` or `source ./catkin_ws/devel_isolated/setup.bash` if compiled isolatedly. 

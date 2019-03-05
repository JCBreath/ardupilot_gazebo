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
sim_vehicle.py -v ArduCopter -f gazebo-iris  -m --mav10 -I0 --add-param-file="$(rospack find ardupilot_gazebo)/sitl_parameters/gps.parm"
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
sim_vehicle.py -v ArduCopter -f gazebo-iris  -m --mav10 -I0 --add-param-file="$(rospack find ardupilot_gazebo)/sitl_parameters/gps.parm"
```
```
sim_vehicle.py -v ArduCopter -f gazebo-iris  -m --mav10 -I1 --add-param-file="$(rospack find ardupilot_gazebo)/sitl_parameters/gps.parm"
```

Launch the multi-agent gazebo file
```
roslaunch ardupilot_gazebo multi_agent_iris.launch
```

### Explore simple commands with mavros

Start the simulator and launch the gazebo environment (with or without simulated optitrack, see above)

Run the example script:
```
rosrun ardupilot_gazebo mavros_control.py
```

For more information, see `src/mavros_control.py`

## Troubleshooting

### Missing libArduPilotPlugin.so ... etc

In case you see this message when you launch gazebo with demo worlds, check you have no error after sudo make install.  
If no error use "ls" on the install path given to see if the plugin is really here.  
If this is correct, check with "cat /usr/share/gazebo/setup.sh" the variable GAZEBO_PLUGIN_PATH. It should be the same as the install path. If not use "cp" to copy the lib to right path. 

For Example

```
sudo cp -a /usr/lib/x86_64-linux-gnu/gazebo-7.0/plugins/ /usr/lib/x86_64-linux-gnu/gazebo-7/
```

path mismatch is confirmed as ROS's glitch. It'll be fixed.

### Unstable behavior

Try reducing the roll rate D-gain:
```
> param set ATC_RAT_RLL_D 0.000100
```

### Simulating Indoor GPS with optitrack

In the simulation, make sure GPS and compass are off
```
> param set GPS_TYPE 0
> param set EK2_GPS_TYPE 3
> param set COMPASS_USE 0
> param set COMPASS_USE2 0
> param set COMPASS_USE3 0
```

### Future(not activated yet)
To use Gazebo gps, you must offset the heading of +90° as gazebo gps is NWU and ardupilot is NED 
(I don't use GPS altitude for now)  
example : for SITL default location
```
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>-35.363261</latitude_deg>
      <longitude_deg>149.165230</longitude_deg>
      <elevation>584</elevation>
      <heading_deg>87</heading_deg>
    </spherical_coordinates>
```


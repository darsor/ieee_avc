# ieee_avc  
### A repository for the USU AVC Team launch files and configurations  

This includes 5 ros packages: four for the VESC (`vesc_driver`, which communicates with the VESC, `vesc_ackerman`, which converts between VESC and ackermann ROS messages, `vesc_msgs`, which contains definitions for the VESC messages, and `ackermann_cmd_mux`, which is an input multiplexer for ackermann commands), and one called `avc`. The `avc` package contains all the launch and configuration files we are using, and it is the focus of this documentation.  

Here is a [diagram](https://drive.google.com/open?id=0B6Ak-1eCXMiBMmItdGhSRnpRVDQ) of the system setup.  

### Table of Contents  
1. [Directory Tree](https://github.com/darsor/ieee_avc#directory-tree-of-the-avc-package)  
2. [Launch Files](https://github.com/darsor/ieee_avc#launch-files)  
  2.1 [Launch File Locations](https://github.com/darsor/ieee_avc#launch-file-locations)  
  2.2 [How to use the Launch Files](https://github.com/darsor/ieee_avc#how-to-use-the-launch-files)  
  2.3 [The Master Launch File](https://github.com/darsor/ieee_avc#the-master-launch-file)  
  2.4 [Recording Bag Files](https://github.com/darsor/ieee_avc#recording-bag-files)  
3. [Configuration Files](https://github.com/darsor/ieee_avc#configuration-files)  
  3.1 [Config File Locations](https://github.com/darsor/ieee_avc#config-file-locations)  
  3.2 [Config File Descriptions](https://github.com/darsor/ieee_avc#config-files)  

## Directory tree of the `avc` package
```
avc                                           <-- The package root directory 
├── CMakeLists.txt  
├── config                                    <-- The configuration files (used by launch files)  
│   ├── amcl.yaml  
│   ├── ekf_global.yaml  
│   ├── ekf_local.yaml  
│   ├── gps.yaml  
│   ├── imu.yaml  
│   ├── joy_teleop.yaml  
│   ├── laser.yaml  
│   ├── mux.yaml  
│   └── vesc.yaml  
├── launch                                    <-- The launch files
│   ├── amcl.launch  
│   ├── includes                              <-- Modular XML files used by the launch files
│   │   ├── amcl  
│   │   │   ├── amcl.launch.xml  
│   │   │   └── map.launch.xml  
│   │   ├── inputs  
│   │   │   └── joy_teleop.launch.xml  
│   │   ├── localization  
│   │   │   ├── ekf_global_localization.launch.xml  
│   │   │   ├── ekf_local_localization.launch.xml  
│   │   │   └── navsat_transform.launch.xml  
│   │   ├── sensors  
│   │   │   ├── sensors.launch.xml  
│   │   │   └── static_transforms.launch.xml  
│   │   └── vesc  
│   │       └── vesc.launch.xml  
│   ├── localization.launch  
│   ├── master.launch  
│   ├── sensors.launch  
│   └── teleop.launch  
├── map                                       <-- The map and its configuration
│   ├── map.pgm  
│   └── map.yaml  
└── package.xml  
```
  
## Launch files  
The launch files are the user front-end for making things happen on the car. They are written in xml syntax. You'll notice that there is an `includes` folder inside the launch file directory. Inside there are several *.xml files (e.g. 'sensors.launch.xml'). Each one performs a very specific function, such as launching the sensor nodes, launching the localization nodes, or publishing static transforms. These are included and used by the higher-level launch files (the ones in the `launch` directory). That way, when creating a new launch file, you can include the files that do the things you want instead of re-writting all the code each time.  

### Launch file locations:  
Launch files are located in the `ieee_avc/src/avc/launch/` directory. (Use `rosfind avc/launch` to get there)  

### How to use the launch files:  
+ `roslaunch avc sensors.launch` launches only the sensors (GPS, IMU, and Lidar) with their static_transforms
+ `roslaunch avc teleop.launch` launches teleop (joystick controller and vesc to drive the car around)
+ `roslaunch avc localization.launch` launches global and local localization nodes  
+ `roslaunch avc amcl.launch` launches amcl and map_server nodes  

Several of these launch files can be run simultaneously (for example, if you want to run teleop with sensors, you could run both of the launch files).  


### The master launch file:  
There is one more launch file called `master.launch`. This launch file is configurable via command line to run whatever you want. For example, running  

```roslaunch avc master.launch teleop:=true vesc:=true sensors:=true local_localization:=true```  

will start the teleop, launch the sensor nodes, and start just the local localization node.  
Here is a list of possible command line arguments:  
+ `teleop:=true` starts the joystick nodes that provide teleop commands (does **not** start the vesc driver, only the joystick)  
+ `vesc:=true` starts the vesc driver node and ackermann command multiplexer (for example, use with the `teleop` option to be able to drive the car around with the controller.  
+ `sensors:=true` starts the IMU, GPS, and Lidar nodes  
+ `localization:=true` starts local and global localization nodes (undefined behavior if used with the individual local or global localization arguments)  
+ `local_localization:=true` starts only the local localization node (undefined behavior if used with any other localization arguments)  
+ `global_localization:=true` starts only the global localization node (undefined behavior if used with any other localization arguments)  
+ `amcl:=true` starts the amcl node  
+ `map_server:=true` starts the map_server node  

### Recording bag files
All launch files can be used to create bag files. Simply add the `bag:=true` argument when you launch it. (NOTE: this only needs to be done once, so if you start multiple launch files then only use the argument once). Once the launched nodes are killed, the .bag file can be found in the `~/.ros` directory.
  
  
## Configuration files  
Configuration files are all *.yaml files, and contain the parameters that are used by the launchfiles.  
### Config file locations:  
Launch files are located in the `ieee_avc/src/avc/config/` directory. (Use `rosfind avc/config` to get there). The only exception to this is the `map.yaml` file, which is located in the `map` directory of the `avc` package.
### Config files: 
+ `laser.yaml` configuration for the Lidar ([link](http://wiki.ros.org/sick_tim))  
+ `gps.yaml` configuration parameters for the GPS ([link](http://wiki.ros.org/nmea_navsat_driver#Parameters-1))  
+ `imu.yaml` configuration parameters for the IMU ([link](http://wiki.ros.org/razor_imu_9dof#Sensor_Calibration))  
+ `vesc.yaml` configuration for the VESC  
+ `amcl.yaml` filter parameters for the AMCL algorithm ([link](http://wiki.ros.org/amcl#Parameters))  
+ `ekf_global.yaml` configuration parameters for the global localization algorithm ([link](http://wiki.ros.org/robot_localization#Parameters))  
+ `efk_local.yaml` configuration parameters for the local localization algorithm ([link](http://wiki.ros.org/robot_localization#Parameters))  
+ `joy_teleop.yaml`  configuration for the node that transforms the joystick messages into ackermann messages ([link](http://wiki.ros.org/joy_teleop#Parameters))  
+ `mux.yaml` configuration for the ackermann command multiplexer  
+ `map.yaml` map parameters (automatically generated with the map) ([link](http://wiki.ros.org/map_server#YAML_format))  

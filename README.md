# ieee_avc  
## A repository for AVC Team launch files and configurations  

This includes code for controlling the VESC and converting Ackermann messages.  

### To use the launchfiles:  
`roslaunch avc teleop.launch` launches joystick teleop  
`roslaunch avc sensors.launch` launches only the sensors (GPS, IMU, and Lidar)  
`roslaunch avc teleop_with_sensors.launch` launches teleop with sensors enabled  
`roslaunch avc teleop_with_localization.launch` launches joystick teleop and starts localization nodes  

### Configuration files  
Configuration files are stored in the `config` directory of the AVC package. They are all *.yaml files, and contain the parameters that will be used by the launchfiles.  

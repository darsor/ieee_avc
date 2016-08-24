# ieee_avc  
### A repository for the USU AVC Team launch files and configurations  

This includes code for controlling the VESC and converting Ackermann messages.  

## Launch files  
The launch files are the user front-end for making things happen on the car. They are written in xml syntax. You'll notice that there is an `includes` folder inside the launch file directory. Inside there are several *.xml files (e.g. 'sensors.launch.xml'). Each one performs a very specific function, such as launching the sensor nodes, launching the localization nodes, or publishing static transforms. These are included and used by the higher-level launch files (the ones in the `launch` directory). That way, when creating a new launch file, you can include the files that do the things you want instead of re-writting all the code each time.  
### Launch file locations:  
Launch files are located in the `ieee_avc/src/avc/launch/` directory. (Use `rosfind avc/launch` to get there)  
### How to use the launch files:  
`roslaunch avc sensors.launch` launches only the sensors (GPS, IMU, and Lidar) with their static_transforms
`roslaunch avc teleop.launch` launches teleop (joystick controller and vesc to drive the car around)
`roslaunch avc localization.launch` launches global and local localization nodes  
`roslaunch avc amcl.launch` launches global and local localization nodes  

### Configuration files  
Configuration files are stored in the `config` directory of the AVC package. They are all *.yaml files, and contain the parameters that will be used by the launchfiles.  

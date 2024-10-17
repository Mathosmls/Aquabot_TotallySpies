robot_localization
==================

robot_localization is a package of nonlinear state estimation nodes. The package was developed by Charles River Analytics, Inc.

Please see documentation here: http://wiki.ros.org/robot_localization

For the aquabot challenge : 

This node fuses the IMU and GPS sensors data (gps data which is transformed in a cartesian coordinate system).

To launch (ideally after you launched the aquabot simulation anf after building the ws) :
```
source /opt/ros/humble/setup.bash 
. install/setup.bash 
ros2 launch robot_localization ekf.launch.py
```

# sensors_processing

This package corrects the transforms of the "aquabot" package. It also launches Rviz2 and adds covariances to the IMU and GPS messages.

**14/11:** Added a low pass filter to odometry position and velocity messages.

## Initialization
To initialize the environment, run the following commands:
```
source /opt/ros/humble/setup.bash 
. install/setup.bash
```


## Starting the Node

To run the control node, execute the following commands in order (other simulation modes are supported):
```
ros2 launch aquabot_gz competition.launch.py world:=aquabot_regatta

ros2 launch sensors_processing sensors_processing.launch.py
```
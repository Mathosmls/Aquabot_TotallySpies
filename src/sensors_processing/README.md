This package corrects the transforms of the package "aquabot". It also launches Rviz2 and add covariances to the IMU and GPS messages.


To launch (ideally after you launched the aquabot simulation anf after building the ws) :

```
source /opt/ros/humble/setup.bash 
. install/setup.bash 
ros2 launch sensors_processing sensors_processing.launch.py 
```

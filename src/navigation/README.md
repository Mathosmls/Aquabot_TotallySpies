
This package is kind of a testing package. 
For now it subscribes to imu and gps topics but does nothing with them.

__But it also reads the keyboard and send the commands to the aquabot's motors.__

To run : 
go to the root of your workspace (vrx_ws) and build :

first terminal (to launch the sim): 
```
source /opt/ros/humble/setup.bash 
colcon build --merge-install
. install/setup.bash 
ros2 launch aquabot_gz competition.launch.py world:=aquabot_regatta
```
Then you have 2 options :

* Option 1 (launches everything at once):
### !!! You have to install xterm before!!!
```
sudo apt-get install xterm
```
2nd terminal :
```
source /opt/ros/humble/setup.bash 
. install/setup.bash 
ros2 launch navigation manual_nav.launch.py
```


* Option 2 :

2nd terminal :
```
source /opt/ros/humble/setup.bash 
. install/setup.bash 
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
3rd terminal :
```
source /opt/ros/humble/setup.bash 
. install/setup.bash 
ros2 run navigation navigation
```



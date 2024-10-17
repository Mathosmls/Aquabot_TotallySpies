For now, can run a simple node that reads the keyboard, the gps and the imu. 
It uses the keyboard's input to control the aquabot's motors.

To run : 
go to the root of your workspace (vrx_ws) and build :

first terminal : 
```
source /opt/ros/humble/setup.bash 
colcon build --merge-install
. install/setup.bash 
ros2 launch aquabot_gz competition.launch.py world:=aquabot_regatta
```
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

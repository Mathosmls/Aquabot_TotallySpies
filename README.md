Ws for the aquabot challenge 2024.

For now to launch everything that Mathis did :

To do in each terminal :
```
source /opt/ros/humble/setup.bash 
. install/setup.bash 
```

Then (preferably in this order): 

```
ros2 launch aquabot_gz competition.launch.py world:=aquabot_regatta
```

```
ros2 launch sensors_processing sensors_processing.launch.py
```

```
ros2 launch robot_localization aquabot_ekfs.launch.py
```

```
ros2 launch nav2aqua nav2aqua.launch.py
```

```
ros2 run py_control controller_node --ros-args -p use_sim_time:=true
```



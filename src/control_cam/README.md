Ws for the aquabot challenge 2024.


To do in each terminal :
```
source /opt/ros/humble/setup.bash 
. install/setup.bash 
```

Attention, se mettre dans le fichier Aquabot/Totally_spies 

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
et dans mon fichier, lancer ma node

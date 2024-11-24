# robot_localization

**robot_localization** est un package de nœuds d'estimation d'état non linéaire.  
Le package a été développé par Charles River Analytics, Inc.

Vous pouvez consulter la documentation ici : [http://wiki.ros.org/robot_localization](http://wiki.ros.org/robot_localization)

## Pour le défi Aquabot :

Ce noeud fusionne les données des capteurs IMU et GPS (les données GPS sont transformées dans un système de coordonnées cartésien).  
Il utilise principalement les données GPS, car elles sont rapides et suffisamment précises (l'IMU a causé des décalages pour des raisons obscures).

## Lancement

Lancer idéalement après avoir démarré la simulation Aquabot et sourcé :

```
source /opt/ros/humble/setup.bash 
. install/setup.bash 
ros2 launch robot_localization ekf.launch.py
```
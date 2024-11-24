# nav2aqua

Ce package utilise 5% des capacités de NAV2 pour créer une carte de coût et un planificateur de trajectoire.

### Démarrage du Node

Pour exécuter le node de contrôle, exécutez les commandes suivantes dans cet ordre (les autres modes de simulations fonctionnent):
```
ros2 launch aquabot_gz competition.launch.py world:=aquabot_regatta

ros2 launch sensors_processing sensors_processing.launch.py

ros2 launch robot_localization aquabot_ekfs.launch.py

ros2 launch nav2aqua nav2aqua.launch.py
```


## Description :

Ce package est un nœud ROS2 qui s'intègre avec la bibliothèque NAV2 pour la planification de trajectoire. Il s'abonne aux topics `/goal_pose` et `/odometry/filtered/map` pour calculer un chemin de la position actuelle du robot vers un objectif spécifié.

### Fonctionnalités principales :
- **Souscription à la position de l'objectif :** S'abonne au topic `/goal_pose` pour recevoir une position cible pour le robot.
- **Souscription à l'odométrie :** S'abonne au topic `/odometry/filtered/map` pour obtenir la position actuelle du robot.
- **Planification de trajectoire :** Utilise l'action `ComputePathToPose` de NAV2 pour calculer un chemin entre la position actuelle du robot et l'objectif.
- **Publication de l'objectif cible :** Publie l'objectif cible sur le topic `/goal_target`.

## Dépendances :
- ROS 2 Humble
- nav2_msgs
- rclcpp
- geometry_msgs
- nav_msgs
- rclcpp_action
- tf2_geometry_msgs

## Utilisation :

Une fois le package lancé, il écoute les messages contenant la pose de l'objectif et les données d'odométrie. Il calcule ensuite le chemin optimal vers l'objectif en utilisant l'action `ComputePathToPose` de NAV2 et publie le résultat.

Exemple :

```
ros2 topic echo /goal_pose geometry_msgs/PoseStamped 
```
  stamp: now
  frame_id: 'map'
pose:
  position:
    x: 10.0
    y: 5.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"

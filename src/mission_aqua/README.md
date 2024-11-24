# mission_aqua

**mission_aqua** est un package qui gère le déroulement du challenge aquabot. Il a besoin de tout le reste du projet pour bien fonctionner.


## Utilisation

Avant de lancer le node, assurez-vous d'avoir configuré votre environnement :
```
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Démarrage du Node

Pour exécuter ::
```
ros2 run mission_aqua mission_aqua_node  --ros-args -p use_sim_time:=true
```




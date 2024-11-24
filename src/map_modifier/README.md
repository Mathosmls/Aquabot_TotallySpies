# map_modifier

**map_modifier** est un package qui fusionne la map créée par map_server and la postion des éoliennes.


## Utilisation

Avant de lancer le node, assurez-vous d'avoir configuré votre environnement :
```
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Démarrage du Node

Pour exécuter le node de contrôle, exécutez les commandes suivantes dans cet ordre (les autres modes de simulations fonctionnent):
```
ros2 run map_modifier map_modifier --ros-args -p use_sim_time:=true
```

## Fonctionnalités

Le node **map_modifier** permet de publier une map de la simulation modifiée grâce au topic '/modified_map', en fonction de la position des éoliennes sur la map d'origine et des orbsatcles.





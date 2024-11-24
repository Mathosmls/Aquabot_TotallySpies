# Navigation

Le package **navigation** est conçu pour transformer les positions GNSS des éoliennes en positions locales.

## Fonctionnalités

- Conversion des coordonnées GNSS en coordonnées locales.
- Publication des positions transformées dans un environnement ROS 2.

## Utilisation

### Exécution du Node

Pour exécuter le node `graph_node`, utilisez la commande suivante :

```
ros2 run navigation graph_node --ros-args -p use_sim_time:=true
```

### Dépendances

Assurez-vous que les dépendances ROS 2 nécessaires sont installées avant de lancer le node.

## Architecture du Code

### Fichiers principaux

- **`graph_node.cpp`** : Implémente la logique de transformation GNSS en coordonnées locales.
- **`graph_node.h`** : Déclare les fonctions et classes utilisées dans le node.

### Points Clés de l'Implémentation

- **Conversion GNSS** : Les positions GNSS des éoliennes sont transformées en coordonnées locales et utilisées dans les algorithmes de navigation.
- **Publication ROS 2** : Les positions converties sont publiées sur des topics ROS appropriés (PoseArray,PoseStamped et PointCloud2).


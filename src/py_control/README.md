
# py_control

`py_control` est un package ROS2 conçu pour le contrôle d'un USV (Unmanned Surface Vehicle) en utilisant un contrôleur MPPI. Ce package permet d'exécuter différents modes de navigation, allant du suivi de points au tracé de cercles.

## Fonctionnalités

Le package propose 5 modes de contrôle, activables via le topic `/mode` :
- **Mode 0 :** Navigation vers un point.
- **Mode 1 :** Suivi rapide d'un chemin.
- **Mode 2 :** Suivi d'une trajectoire circulaire pour la recherche de Qr codes.
- **Mode 3 :** Suivi d'une trajectoire circulaire pour la phase bonus.
- **Mode 4 :** Suivi lent d'un chemin.

## Dépendances

Le package nécessite les éléments suivants pour fonctionner correctement :
- **Nodes requis :** 
  - `aquabot_ekfs` (Estimation de position)
  - `sensors_processing` (Traitement des capteurs)
  - `nav2aqua` (Créer une costmap et un planificateur avec Nav2)
- **ROS 2 Humble :** Assurez-vous que ROS 2 Humble est correctement installé.

## Installation

1. Clonez ce repository dans votre espace de travail ROS2 :
   ```
   git clone <https://github.com/Mathosmls/Aquabot_TotallySpies.git>
   ```
2. Compilez le package :
   ```
   git pull
   colcon build --merge-install
   . install/setup.bash
   ```

## Utilisation

Avant de lancer le node, assurez-vous d'avoir configuré votre environnement :
```
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Démarrage du Node

Pour exécuter le node de contrôle, exécutez les commandes suivantes dans cet ordre (les autres modes de simulations fonctionnent):
```
ros2 launch aquabot_gz competition.launch.py world:=aquabot_regatta

ros2 launch sensors_processing sensors_processing.launch.py

ros2 launch robot_localization aquabot_ekfs.launch.py

ros2 launch nav2aqua nav2aqua.launch.py

ros2 run py_control controller_node --ros-args -p use_sim_time:=true
```

### Configuration du Mode

Le mode peut être changé en publiant un entier sur le topic `/mode` :
- `0` : Navigation vers un point.
- `1` : Suivi rapide d'un chemin.
- `2` : Suivi d'une trajectoire circulaire pour la recherche de Qr codes.
- `3` : Suivi d'une trajectoire circulaire pour la phase bonus.
- `4` : Suivi lent d'un chemin.

Exemple de publication (via `ros2 topic pub`) :
```
ros2 topic pub /mode std_msgs/msg/UInt8 "{data: 1}"
```

### Topics publiés

Le node publie les commandes suivantes pour contrôler les moteurs et leur orientation :
- `/aquabot/thrusters/left/thrust` : Poussée moteur gauche.
- `/aquabot/thrusters/right/thrust` : Poussée moteur droit.
- `/aquabot/thrusters/left/pos` : Angle moteur gauche.
- `/aquabot/thrusters/right/pos` : Angle moteur droit.
- `/goal_pose` : Position cible.

## Architecture du Code

### Fichiers principaux

- **`controller_node.py`** : Implémente le contrôleur principal et gère la logique des différents modes.
- **`mppi_utils`** : Contient les utilitaires pour le contrôleur MPPI (Model Predictive Path Integral).

### Points clés de l'implémentation

1. **Modes de navigation :** Chaque mode est configuré pour des scénarios spécifiques comme le suivi de chemin rapide (Mode 1) ou le suivi de trajectoire circulaire (Modes 2 et 3).
2. **Intégration MPPI :** Utilise des instances configurables de `MPPIControllerForAquabot` pour optimiser les commandes.
3. **Callbacks ROS2 :** Gestion des messages pour les topics clés comme `/odometry/filtered/map/processed`, `/plan`, et `/goal_target`.

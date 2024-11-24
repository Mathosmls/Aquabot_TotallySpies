Ws for the aquabot challenge 2024.

For now to launch everything that Mathis did :

To do in each terminal :
```
source /opt/ros/humble/setup.bash 
. install/setup.bash 
```
!!!!NEW : IF YOU DONT WANT TO DEBUG, YOU CAN LAUNCH sensors_processing, aquabot_ekfs and nav2aqua AT THE SAME TIME !!!!
```
ros2 launch mission sensors_nav.launch.py 
```

But if you want to debug it's easier to do  (preferably in this order): 

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

# Mission Aqua

**Mission Aqua** est un package ROS2 qui gère l'ensemble de la mission d'inspection des éoliennes dans le cadre du concours. Ce noeud orchestre le parcours, l'identification et l'inspection des éoliennes en exploitant plusieurs capteurs et algorithmes.

## Fonctionnalités

1. **Planification de Trajectoire** :
   - Utilise un solveur TSP (Traveling Salesperson Problem) pour calculer l'ordre optimal des éoliennes à visiter.
   - Génère des points cibles ajustés autour des éoliennes pour faciliter les manœuvres.

2. **Inspection des Éoliennes** :
   - Identification des éoliennes par QR code.
   - Détection des positions critiques pour inspection grâce aux capteurs acoustiques.

3. **Phases de la Mission** :
   - **Phase 1** : Exploration des éoliennes dans l'ordre optimal.
   - **Phase 2** : Identification de l'éolienne critique à inspecter.
   - **Phase 3** : Inspection approfondie autour de l'éolienne critique.

4. **Gestion des Modes** :
   - Contrôle de la vitesse et des manœuvres en fonction de la proximité des cibles.

## Dépendances

Le package dépend des bibliothèques et packages suivants :
- **ROS 2 Humble** ou supérieur.
- `numpy`
- `tf_transformations`
- `nav_msgs`
- `geometry_msgs`
- `std_msgs`
- `rclpy`
- `ros_gz_interfaces`

Assurez-vous d'avoir installé toutes les dépendances nécessaires avant d'exécuter le noeud.

## Structure du Noeud

### Publications
- `/goal_target` (`geometry_msgs/PoseStamped`) : Envoie la position cible pour atteindre une éolienne.
- `/goal_pose` (`geometry_msgs/PoseStamped`) : Envoie une position spécifique pour le déplacement.
- `/mode` (`std_msgs/UInt8`) : Change le mode de fonctionnement (vitesse, inspection, etc.).
- `/current_wt` (`geometry_msgs/Point`) : Indique la position actuelle de l'éolienne cible.

### Subscriptions
- `/local_wind_turbine_positions` (`geometry_msgs/PoseArray`) : Reçoit les positions initiales des éoliennes.
- `/pos_gps_align_qr` (`geometry_msgs/PoseStamped`) : Reçoit les positions GPS alignées avec les QR codes.
- `/odometry/filtered/map/processed` (`nav_msgs/Odometry`) : Reçoit la position actuelle du robot.
- `/vrx/windturbinesinspection/windturbine_checkup` (`std_msgs/String`) : Reçoit les informations des éoliennes identifiées.
- `/aquabot/sensors/acoustics/receiver/range_bearing` (`ros_gz_interfaces/ParamVec`) : Données acoustiques pour localiser les éoliennes critiques.

## Installation

1. Clonez ce dépôt dans votre espace de travail ROS2 :
   ```bash
   cd ~/ros2_ws/src
   git clone <url_du_dépôt>



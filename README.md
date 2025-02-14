# Aquabot Challenge 2024 - Workspace des TotallySpies
Pour lancer nos codes, référez vous à la section **Lancer les nodes**
## Description
Ce projet est notre workspace principal pour la **Aquabot Challenge 2024**. Vous pourrez y trouver les différents packages et nodes nécessaires à la simulation et au contrôle de l’Aquabot. Ce fichier README décrit les étapes pour configurer et lancer les différentes parties du système.

---

## Installation et Configuration
### Pré-requis
- ROS 2 Humble installé 

- Cloner le repository du projet :
```
git clone https://github.com/Mathosmls/Aquabot_TotallySpies.git
```

- Compiler le workspace


### Sourcing des environnements
Avant de lancer les nodes, assurez-vous d'avoir configuré les environnements dans chaque terminal :
```
source /opt/ros/humble/setup.bash 
. install/setup.bash
```

---

## Lancer les Nodes

# Lancer tout le projet
!!!!!!!Avant de lancer nos codes, il est fortement recommandé de lancer une simulation avant!!!!! 
## !!!!! Commandes à éxécuter dans cette ordre pour être sûr que tout se passe bien!!!!! :
```
ros2 launch mission_aqua controller_wt_cam.launch.py 
ros2 launch mission_aqua nav_localization.launch.py 
ros2 run mission_aqua mission_aqua_node --ros-args -p use_sim_time:=true
```

---

### Lancer les Nodes séparément (pour débogage)
Pour un contrôle plus précis et pour déboguer, lancez les nodes dans cet ordre recommandé :

1. **Simulation Gazebo avec les différents mondes, ici, on prend par exmple `aquabot_regatta` :**
```
ros2 launch aquabot_gz competition.launch.py world:=aquabot_regatta
```

2. **Traitement des données des capteurs :**
```
ros2 launch sensors_processing sensors_processing.launch.py
```

3. **Filtrage Kalman étendu (EKF) pour la localisation :**
```
ros2 launch robot_localization aquabot_ekfs.launch.py
```

4. **Navigation avec Nav2 :**
```
ros2 launch nav2aqua nav2aqua.launch.py
```

5. **Contrôleur :**
```
ros2 run py_control controller_node --ros-args -p use_sim_time:=true
```

---

## Structure des Packages et Nodes
### Packages principaux
- **`sensors_processing`** : Pré-traitement des données capteurs (IMU, GPS, etc.).
- **`robot_localization`** : Estimation d’état à l’aide de EKFs.
- **`nav2aqua`** : Stack de navigation Nav2 adaptée à l’Aquabot.
- **`py_control`** : Contrôleur MPPI pour la gestion des différents mouvements de l'Aquabot (suivi de chemin, d'une trajectoire circulaire...).
- **`control_cam_qr`** : Analyse des données de la caméra, détection des QR codes .

# control_cam_qr

Le package **control_cam_qr** combine le contrôle de la caméra et le décodage des QR codes pour interagir avec des éoliennes et ajuster la position de la caméra en conséquence. Il permet également de renvoyer la position GPS calculée pour que le robot soit en face du QR code.

## Prérequis

Avant d'utiliser ce package, assurez-vous d'avoir installé la bibliothèque **opencv-python**. Vous pouvez l'installer avec la commande suivante :
```
pip install opencv-python
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

ros2 run control_cam_qr control_cam_qr_node

```


## Fonctionnalités

Le node **control_cam_qr_node** combine plusieurs tâches essentielles :

- **Contrôle de la caméra :** Ajuste l'angle de la caméra pour viser un QR code spécifique à partir de la position du robot et de la cible.
- **Décodage des QR codes :** Scanne les QR codes à partir du flux vidéo de la caméra et répertorie l'id et l'état des nouvelles éoliennes.
- **Publication des données :** Publie l'id et l'état des nouvelles éoliennes ainsi que la position GPS pour s'aligner avec le QR code afin de mieux le détecter.

### Topics utilisés

- **/aquabot/thrusters/main_camera_sensor/pos** : Publie l'angle ajusté de la caméra.
- **/vrx/windturbinesinspection/windturbine_checkup** : Publie l'état des éoliennes sous forme de JSON.
- **pos_gps_align_qr** : Publie la position GPS pour aligner le robot avec le QR code quand la caméra est alignée vers la proue du bateau.
- **/aquabot/sensors/cameras/main_camera_sensor/image_raw** : Topic permettant d'obtenir les images de la caméra.
- **/odometry/filtered/map** : Topic permettant d'obtenir la position du robot.
- **/current_wt** : Topic permettant d'obtenir la position cible des éoliennes.

## Fonctionnement

- Le node traite les images provenant de la caméra du robot pour détecter les QR codes.
- Lorsqu'un nouveau QR code est détecté, il extrait l'ID et l'état de l'éolienne et les publie.
- Il calcule ensuite l'angle nécessaire pour aligner la caméra avec la cible (le QR code).
- Enfin, il ajuste l'orientation de la caméra en fonction de la position du robot et de la cible.

## Code Principal

Le fichier principal du node est **control_cam_qr_node.py**. Il contient la logique pour :

1. Gérer la caméra et son angle d'orientation.
2. Décoder les QR codes et mettre à jour l'état des éoliennes.
3. Publier la position GPS alignée pour une détection optimale du QR code.



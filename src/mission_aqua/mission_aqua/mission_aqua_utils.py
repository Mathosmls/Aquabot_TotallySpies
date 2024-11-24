#UTILS FOR THE NODE MISSION_AQUA

import numpy as np
from itertools import permutations
from tf_transformations import euler_from_quaternion
import math



def Quat2yaw(quat_msg) :
    quaternion = (  quat_msg.x,
                    quat_msg.y,
                    quat_msg.z,
                    quat_msg.w)
    roll, pitch, yaw = euler_from_quaternion(quaternion)
    yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
    return yaw

def calculate_distance(p1, p2):
    """Calculer la distance Euclidienne entre deux points."""
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def calculate_distance_matrix(positions):
    """
    Calcule la matrice des distances entre toutes les positions.
    :param positions: Array 2D de shape (n, 2) où n est le nombre de positions.
    :return: Matrice NumPy où element (i, j) représente la distance entre position[i] et position[j].
    """
    diff = positions[:, np.newaxis, :] - positions[np.newaxis, :, :]
    distance_matrix = np.sqrt(np.sum(diff**2, axis=-1))
    return distance_matrix

def tsp_solver(start_position, target_positions):
    """
    Résout le problème du voyageur de commerce en utilisant des tableaux NumPy
    pour trouver l'ordre optimal des positions à visiter.
    :param start_position: Array de shape (1, 2) représentant la position de départ.
    :param target_positions: Array 2D de shape (n, 2) représentant les positions cibles.
    :return: Liste des positions dans l'ordre optimal et la distance minimale.
    """
    # Combiner les positions en un seul tableau
    positions = np.vstack((start_position, target_positions))

    # Calculer la matrice des distances
    distance_matrix = calculate_distance_matrix(positions)

    # Obtenir le nombre de positions
    num_positions = len(positions)

    # Variables pour le suivi de la meilleure solution
    min_distance = float('inf')
    best_order = None

    # Générer toutes les permutations des indices des cibles uniquement
    target_indices = np.arange(1, num_positions)  # Indices des cibles
    perms = permutations(target_indices)

    for perm in perms:
        # Créer l'ordre des positions en ajoutant le départ au début
        order = (0,) + perm
        distance = sum(distance_matrix[order[i], order[i+1]] for i in range(len(order) - 1))
        
        if distance < min_distance:
            min_distance = distance
            best_order = order

    # Obtenir les positions dans l'ordre optimal
    optimal_path = positions[np.array(best_order)] 
    optimal_path =optimal_path[1:]
    return optimal_path, min_distance



def calculate_offset_target(robot_position, target_position, radius=5, angle_offset=-math.pi/2):
    """
    Calcule une position cible située sur un cercle de rayon spécifié autour de la cible,
    avec un décalage angulaire donné.
    :param robot_position: Tuple (x, y) représentant la position actuelle du robot.
    :param target_position: Tuple (x, y) représentant la position de la cible (ex : éolienne).
    :return: Tuple (x_target, y_target) représentant la position cible finale.
    """
    x_robot, y_robot = robot_position[0],robot_position[1]
    x_target, y_target = target_position[0],target_position[1]

    # Calculer la direction actuelle (vecteur entre le robot et la cible)
    dx = x_target - x_robot
    dy = y_target - y_robot
    # print(dx,dy)

    # Calculer l'angle initial du vecteur direction
    initial_angle = math.atan2(dy, dx)

    # Appliquer le décalage angulaire
    final_angle = initial_angle + angle_offset

    # Calculer les coordonnées de la position cible
    new_x = x_target + radius * math.cos(final_angle)
    new_y = y_target + radius * math.sin(final_angle)

    angle_to_target = math.atan2(new_y - y_robot, new_x - x_robot)

    return np.array([new_x, new_y,angle_to_target])

def extract_id(message):
    try:
        # Nettoyer la chaîne en retirant les accolades et les guillemets doubles
        cleaned_message = message.strip().strip('{}').replace('"', '')
        
        # Diviser en paires clé-valeur
        pairs = cleaned_message.split(',')

        # Parcourir chaque paire
        for pair in pairs:
            # Trouver le premier ":" pour séparer la clé de la valeur
            key, value = pair.split(':', 1)  # Limite à un seul split
            if key.strip() == "id":
                return int(value.strip())  # Retourne l'ID comme un entier
        
        # Si "id" n'est pas trouvé
        raise ValueError("Champ 'id' non trouvé dans le message.")
    except Exception as e:
        raise ValueError(f"Erreur lors de l'extraction de l'ID : {e}")




def update_order(wind_turbines, turbine_id, new_order):
    """
    Met à jour l'ordre de l'éolienne spécifiée dans le dictionnaire.
    
    :param wind_turbines: Dictionnaire contenant les éoliennes et leurs données
    :param turbine_id: ID de l'éolienne à mettre à jour
    :param new_order: Nouvelle valeur pour l'ordre
    :return: None
    """
    if turbine_id in wind_turbines:
        wind_turbines[turbine_id]["order"] = new_order
        # print(f"L'ordre de l'éolienne {turbine_id} a été mis à jour à {new_order}.")
    else:
        print(f"Update order : Éolienne avec ID {turbine_id} introuvable.")

def add_or_update_turbine(turbine_id, wind_turbines_dic, position=(0.0, 0.0), order=0, pos_qr=np.array([0.0, 0.0, 0.0])):
    """
    Ajoute une nouvelle éolienne ou met à jour une éolienne existante.

    Si l'éolienne existe déjà, seule la position pour le QR code (pos_qr) est mise à jour.

    :param turbine_id: ID unique de l'éolienne.
    :param wind_turbines_dic: Dictionnaire des éoliennes {id: {"position": (x, y), "order": int, "pos_qr": np.array}}.
    :param position: Position initiale de l'éolienne (par défaut : (0, 0)).
    :param order: Ordre initial de l'éolienne (par défaut : 0).
    :param pos_qr: Position pour le QR code (par défaut : np.array([0, 0, 0])).
    :return: Le dictionnaire mis à jour.
    """
    
    pos_qr_local = np.copy(pos_qr)
    already_checked=False
    if turbine_id not in wind_turbines_dic:
        # Ajouter une nouvelle éolienne
        wind_turbines_dic[turbine_id] = {
            "position": position,
            "order": order,
            "pos_qr": pos_qr_local
        }
        print(f"Nouvelle éolienne ajoutée : ID {turbine_id}, Position {position}, Ordre {order}, Pos QR {pos_qr_local}")
    else:
        print(f"Éolienne avec ID {turbine_id} existe déjà")
        already_checked=True
    
    return wind_turbines_dic,already_checked

def update_turbine_qr_position(turbine_id, wind_turbines_dic, pos_qr=np.array([0.0, 0.0, 0.0])):
    """
    Met à jour uniquement la position du QR code (pos_qr) pour une éolienne existante.

    :param turbine_id: ID unique de l'éolienne.
    :param wind_turbines_dic: Dictionnaire des éoliennes {id: {"position": (x, y), "order": int, "pos_qr": np.array}}.
    :param pos_qr: Nouvelle position pour le QR code (par défaut : np.array([0, 0, 0])).
    :return: Le dictionnaire mis à jour.
    """
    
    if turbine_id in wind_turbines_dic:
        # Vérifie si l'éolienne existe et met à jour uniquement pos_qr
        wind_turbines_dic[turbine_id]["pos_qr"] = np.copy(pos_qr)
        print(f"Position du QR code mise à jour pour l'éolienne ID {turbine_id} : Pos QR {pos_qr}")
    else:
        print(f"Erreur : Éolienne avec ID {turbine_id} introuvable dans le dictionnaire.")
    
    return wind_turbines_dic

def is_within_distance(current_position, target_position, threshold_distance):
    """
    Vérifie si la position actuelle est à une distance inférieure au seuil de la position cible.

    :param current_position: Liste ou tuple représentant la position actuelle [x, y, theta]
    :param target_position: Liste ou tuple représentant la position cible [x, y, theta]
    :param threshold_distance: Distance seuil
    :return: True si la distance est inférieure au seuil, sinon False
    """

    distance = math.sqrt((current_position[0] - target_position[0])**2 +
                         (current_position[1] - target_position[1])**2)

    return distance < threshold_distance

def find_best_matching_wind_turbine_id(boat_position, wind_turbines_dic, target_distance, bearing):
    """
    Trouve l'ID de l'éolienne qui correspond le mieux à une distance donnée.

    :param boat_position: Tuple (x, y) représentant la position du bateau.
    :param wind_turbines_dic: Dictionnaire des éoliennes {id: {"position": (x, y), "order": int}}.
    :param target_distance: Distance cible entre le bateau et l'éolienne.
    :return: ID de l'éolienne correspondante, ou None si aucune correspondance.
    """
    best_id = None
    smallest_difference = float('inf')  # Initialise avec un écart infini

    for turbine_id, turbine_data in wind_turbines_dic.items():
        print("find best")
        print(turbine_id)
        turbine_position = turbine_data["position"]
        # Calculer la distance entre le bateau et l'éolienne
        distance = math.sqrt((boat_position[0] - turbine_position[0])**2 +
                                (boat_position[1] - turbine_position[1])**2)
        
        # Calculer la différence de distance
        distance_difference = abs(distance - target_distance)

        # Calculer l'angle entre le bateau et l'éolienne
        delta_x = turbine_position[0] - boat_position[0]
        delta_y = turbine_position[1] - boat_position[1]
        angle_to_turbine = math.atan2(delta_y, delta_x)
        angle_to_turbine -=boat_position[2]
        # Calculer la différence angulaire (en radians)
        angle_difference = abs((angle_to_turbine-bearing  + math.pi) % (2 * math.pi) - math.pi)
        # Combiner les deux critères : distance et angle
        # Pondération : donner plus d'importance à la distance ou à l'angle selon vos besoins
        
        score = distance_difference + 30*angle_difference  # Simple somme pour l'instant
        print(distance_difference,angle_difference,score)
        # Trouver l'éolienne avec le plus petit score
        if score < smallest_difference:
            smallest_difference = score
            best_id = turbine_id

    return best_id



import math
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

class MPPIControllerForDifferentialDrive:
    def __init__(
            self,
            horizon_step_T: int = 25,
            number_of_samples_K: int = 500,
            sigma: np.ndarray = np.array([0.4,0.4]),
            param_lambda: float = 1.0,
            stage_cost_weight: np.ndarray = np.array([50.0, 50.0,1]),  # poids pour [x, y, theta]
            v_max: float = 1.0,  # Limite pour la vitesse linéaire maximale
            w_max: float = 1.0,   # Limite pour la vitesse angulaire maximale
            a_v_max: float = 0.6,  # Limite pour l'accélération linéaire maximale
            a_w_max: float = 0.6,   # Limite pour l'accélération angulaire maximale
            dt=0.1,
            var=2
    ):
        self.T = horizon_step_T
        self.K = number_of_samples_K
        self.sigma = sigma
        self.param_lambda = param_lambda
        self.stage_cost_weight = stage_cost_weight
        self.u_prev = np.zeros((self.T, var))  # initialisation des commandes précédentes [v, omega]
        self.v_max = v_max
        self.w_max = w_max
        self.a_v_max = a_v_max *dt  # Nouvelle limite d'accélération linéaire
        self.a_w_max = a_w_max *dt  # Nouvelle limite d'accélération angulaire
        self.dt=0.1
        self.var=var

    def _g(self, v: np.ndarray) -> np.ndarray:
        """Sature les commandes pour respecter les vitesses maximales"""
        v[0] = np.clip(v[0], -self.v_max, self.v_max)  # vitesse linéaire
        v[1] = np.clip(v[1], -self.w_max, self.w_max)  # vitesse angulaire
        return v

    def calc_control_input(self, observed_state: np.ndarray, target_state: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        u = self.u_prev
        S = np.zeros(self.K)  # Coût total pour chaque échantillon
        epsilon = np.random.normal(0, self.sigma, (self.K, self.T, self.var))  # Bruit de contrôle

        # Boucle sur les échantillons
        for k in range(self.K):
            v = np.zeros((self.T, self.var))
            x = observed_state

            for t in range(self.T):
                # Détermine si l'on exploite ou explore
                if k < 0.9 * self.K:
                    v[t] = self._g(u[t] + epsilon[k, t])
                else:
                    v[t] = self._g(epsilon[k, t])

                # Met à jour l'état avec le modèle du véhicule
                x = self._dynamics(x, v[t])
                
                # Calcul du coût pour cet état
                S[k] += self._stage_cost(x, target_state)

            # Ajout du coût terminal
            S[k] += self._terminal_cost(x, target_state)

        # Calcul des poids basés sur les coûts
        w = self._compute_weights(S)

        # Calcul du contrôle pondéré par les poids
        w_epsilon = np.sum(w[:, None, None] * epsilon, axis=0)

        # Appliquer le filtre de moyenne mobile
        w_epsilon = self._moving_average_filter(xx=w_epsilon, window_size=5)

        # Mise à jour des commandes avec les valeurs lissées
        u = u + w_epsilon

        # Appliquer la limite d'accélération
        for t in range(self.T):
           u[t] = self._apply_acceleration_limit(u[t], self.u_prev[t])

        # Mise à jour de u_prev
        self.u_prev[:-1] = u[1:]  # Décale les commandes vers la gauche
        self.u_prev[-1] = u[-1]

        return self._g(u[0]), u



    def _apply_acceleration_limit(self, u: np.ndarray, u_prev: np.ndarray) -> np.ndarray:
        """Applique une limite d'accélération aux commandes"""
        delta_u = u - u_prev  # Calcul de la variation de commande
        
        # Applique une limite à la variation de vitesse linéaire
        if abs(delta_u[0]) > self.a_v_max:
            delta_u[0] = np.sign(delta_u[0]) * self.a_v_max
        
        # Applique une limite à la variation de vitesse angulaire
        if abs(delta_u[1]) > self.a_w_max:
            delta_u[1] = np.sign(delta_u[1]) * self.a_w_max
        
        return u_prev + delta_u

    def _dynamics(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        x, y, theta = state
        v, omega = control
        x += v * math.cos(theta) * self.dt  # pas de temps fixé à self.dts
        y += v * math.sin(theta) * self.dt
        theta += omega * self.dt
        theta=(theta + np.pi) % (2 * np.pi) - np.pi
        return np.array([x, y, theta])

    def _stage_cost(self, state: np.ndarray, target: np.ndarray) -> float:
        x, y, theta = state
        x_t, y_t, theta_t = target
        return (self.stage_cost_weight[0] * (x - x_t)**2 +
                self.stage_cost_weight[1] * (y - y_t)**2 +
                self.stage_cost_weight[2] * (theta - theta_t)**2)

    def _terminal_cost(self, state: np.ndarray, target: np.ndarray) -> float:
        return self._stage_cost(state, target)

    def _compute_weights(self, S: np.ndarray) -> np.ndarray:
        rho = S.min()
        eta = np.sum(np.exp(-1.0 / self.param_lambda * (S - rho)))
        return np.exp(-1.0 / self.param_lambda * (S - rho)) / eta

    def _moving_average_filter(self, xx: np.ndarray, window_size: int) -> np.ndarray:
        # Nombre de lignes et colonnes
        n_rows, n_cols = xx.shape
        
        # Initialisation de la sortie
        xx_mean = np.zeros_like(xx)
        
        # Calcul de la somme cumulative sur les lignes
        cumsum = np.cumsum(xx, axis=0)
        
        # Appliquer la moyenne mobile en utilisant la somme cumulative
        xx_mean[window_size - 1:] = (cumsum[window_size - 1:] - cumsum[:-window_size + 1]) / window_size
        
        # Traiter le début du tableau où la taille de la fenêtre n'est pas complète
        for i in range(1, window_size):
            xx_mean[i - 1] = cumsum[i - 1] / i
        
        return xx_mean


# Fonction pour générer les points de passage en cercle
def generate_circular_waypoints(radius=1.5, num_points=20):
    waypoints = []
    for i in range(num_points):
        angle = 2 * np.pi * i / num_points  # Angle en radians
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        theta = angle + np.pi / 2  # Orientation tangentielle
        theta=(theta + np.pi) % (2 * np.pi) - np.pi
        waypoints.append([x, y, theta])
    return np.array(waypoints)

def generate_figure_eight_waypoints(radius=1.5, num_points=40):
    waypoints = []
    half_points = num_points // 2  # Points pour chaque boucle du 8

    # Boucle de gauche
    for i in range(half_points):
        angle = 2*np.pi * i / (half_points - 1)  # Angle en radians pour la demi-cercle
        x = -radius + radius * np.cos(angle)  # Décalage vers la gauche
        y = radius * np.sin(angle)
        theta = angle + np.pi / 2  # Orientation tangentielle
        theta = (theta + np.pi) % (2 * np.pi) - np.pi
        waypoints.append([x, y, theta])

    # Boucle de droite
    for i in range(half_points):
        angle = 2*np.pi * (i / (half_points - 1))  # Angle en radians pour la demi-cercle
        x = radius - radius * np.cos(angle)  # Décalage vers la droite
        y = -radius * np.sin(angle)
        theta = angle - np.pi / 2  # Orientation tangentielle
        theta = (theta + np.pi) % (2 * np.pi) - np.pi
        waypoints.append([x, y, theta])

    return np.array(waypoints)


def generate_natural_figure_eight_waypoints(a=2.0, num_points=150):
    """
    Génère des waypoints en forme de 8 naturel (lemniscate de Bernoulli) avec une largeur donnée.
    
    :param a: Largeur de la courbe en 8.
    :param num_points: Nombre de points pour décrire la trajectoire.
    :return: Un tableau numpy de waypoints avec [x, y, theta].
    """
    waypoints = []
    for i in range(num_points):
        t = 2 * np.pi * i / num_points  # Paramètre pour générer les points le long de la courbe
        x = a * np.sin(t)
        y = a * np.sin(t) * np.cos(t)
        theta = np.arctan2(y - waypoints[-1][1], x - waypoints[-1][0]) if waypoints else 0
        waypoints.append([x, y, theta])
        
    return np.array(waypoints)



# Fonction de simulation avec affichage de la trajectoire
def run_simulation_mppi_differential_drive():
    # Générer les points de passage en cercle
    waypoints = generate_natural_figure_eight_waypoints(a=3.0, num_points=120)

    # Configuration de la simulation
    horizon_step_T = 20
    sim_steps = 50  # Nombre total d'étapes de simulation

    # Initialisation de l'état du véhicule et du contrôleur
    initial_state = np.array([1.0, 0.0, 0.0])  # Démarre au bas du cercle
    controller = MPPIControllerForDifferentialDrive(horizon_step_T=horizon_step_T)

    # Enregistrement des données pour l'affichage
    states = [initial_state]
    controls = []
    current_state = initial_state
    current_waypoint_index = 0  # Indice du waypoint actuel

    # Boucle de simulation
    for i in range(sim_steps):
        # Obtenir le waypoint actuel
        target_state = waypoints[current_waypoint_index]

        # Vérifier si le véhicule est suffisamment proche du waypoint
        distance_to_waypoint = np.linalg.norm(current_state[:2] - target_state[:2])
        if distance_to_waypoint < 0.1:  # Seuil de proximité

            current_waypoint_index = (current_waypoint_index + 1) %len(waypoints) # Passer au prochain waypoint
            # if current_waypoint_index ==0 or current_waypoint_index ==1 or current_waypoint_index >=39:
            #     current_waypoint_index=2
            print(current_waypoint_index,waypoints[current_waypoint_index])


        # Calculer le contrôle pour atteindre le waypoint actuel
        control, _ = controller.calc_control_input(current_state, target_state)
        current_state = controller._dynamics(current_state, control)
        states.append(current_state)
        print(current_state[2])
        controls.append(control)

    # Extraction des données pour les graphiques
    x_vals = [state[0] for state in states]
    y_vals = [state[1] for state in states]
    theta_vals = [state[2] for state in states]
    
    v_controls = [control[0] for control in controls]
    omega_controls = [control[1] for control in controls]

    # Création des graphiques
    plt.figure(figsize=(12, 6))

    # Affichage de la trajectoire
    plt.subplot(1, 2, 1)
    plt.plot(x_vals, y_vals, label="Trajectoire")
    plt.scatter(waypoints[:, 0], waypoints[:, 1], color='red', label="Waypoints", zorder=5)
    plt.xlabel("Position x (m)")
    plt.ylabel("Position y (m)")
    plt.title("Trajectoire du véhicule")
    plt.legend()
    plt.grid()

    # Affichage des commandes
    plt.subplot(1, 2, 2)
    plt.plot(v_controls, label="Vitesse linéaire (v)")
    plt.plot(omega_controls, label="Vitesse angulaire (omega)")
    plt.xlabel("Étape de simulation")
    plt.ylabel("Commande")
    plt.title("Commandes de contrôle")
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.show(block=True)

if __name__ == "__main__":
    run_simulation_mppi_differential_drive()

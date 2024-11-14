import numpy as np
from numba import jit
import matplotlib.pyplot as plt
from typing import Tuple
import cProfile
import time
from numba import prange

class MPPIControllerForAquabot:
    def __init__(
            self,
            horizon_step_T: int = 25,
            number_of_samples_K: int = 500,
            sigma: np.ndarray = np.array([50, 50, 0.15, 0.15]),
            param_lambda: float = 1.0,
            stage_cost_weight: np.ndarray = np.array([50, 50, 0.0]),  # poids pour [x, y, theta]
            v_max: float = 30.0,  # Limite pour la vitesse linéaire maximale
            w_max: float = 10.3,  # Limite pour la vitesse angulaire maximale
            a_v_max: float = 10.0,  # Limite pour l'accélération linéaire maximale
            a_w_max: float = 10.0,  # Limite pour l'accélération angulaire maximale
            dt=0.1,
            state_var=4
    ):
        self.T = horizon_step_T
        self.K = number_of_samples_K
        self.sigma = sigma
        self.param_lambda = param_lambda
        self.stage_cost_weight = stage_cost_weight
        self.u_prev = np.zeros((self.T, state_var))  # initialisation des commandes précédentes [v, omega]
        self.v_max = v_max
        self.w_max = w_max
        self.a_v_max = a_v_max * dt  # Nouvelle limite d'accélération linéaire
        self.a_w_max = a_w_max * dt  # Nouvelle limite d'accélération angulaire
        self.dt = 0.1
        self.state_var = state_var

        self.m = 1000.0
        self.Iz = 446.0
        self.xU = 182.0
        self.xUU = 224.0
        self.yV = 183.0
        self.yVV = 149.0
        self.nR = 1199.0
        self.nRR = 979.0
        self.max_thrust = 1000
        self.max_angle = np.pi / 4
        self.pos_mot_x = -3
        self.pos_mot_y = 0.6



    
    def calc_control_input(self, observed_state: np.ndarray, target_state: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        u = self.u_prev
        S = np.zeros(self.K)  # Coût total pour chaque échantillon
        epsilon = np.random.normal(0, self.sigma, (self.K, self.T, self.state_var))  # Bruit de contrôle

        # Boucle sur les échantillons
         
        S=boucle(u,S,self.K,self.T,self.state_var,observed_state,epsilon,self.m, self.xU, self.xUU, self.yV, self.yVV, 
                self.pos_mot_x, self.pos_mot_y, self.nR, self.nRR, self.Iz, self.dt,self.max_thrust,
                self.max_angle,target_state,self.stage_cost_weight,self.a_v_max,self.a_w_max,self.v_max,self.w_max)

           

        # Calcul des poids basés sur les coûts
        w = self._compute_weights(S)

        # Calcul du contrôle pondéré par les poids
        w_epsilon = np.sum(w[:, None, None] * epsilon, axis=0)

        # Appliquer le filtre de moyenne mobile
        w_epsilon = self._moving_average_filter(xx=w_epsilon, window_size=5)

        # Mise à jour des commandes avec les valeurs lissées
        u = u + w_epsilon

        # Mise à jour de u_prev
        self.u_prev[:-1] = u[1:]  # Décale les commandes vers la gauche
        self.u_prev[-1] = u[-1]

        return _g(u[0],self.max_thrust,self.max_angle), u

    def _apply_acceleration_limit(self, u: np.ndarray, u_prev: np.ndarray) -> np.ndarray:
        delta_u = (u - u_prev) / self.dt

        if abs(delta_u[0]) > self.a_v_max:
            delta_u[0] = np.sign(delta_u[0]) * self.a_v_max

        if abs(delta_u[1]) > self.a_w_max:
            delta_u[1] = np.sign(delta_u[1]) * self.a_w_max

        return u_prev + delta_u * self.dt

    def _compute_weights(self, S: np.ndarray) -> np.ndarray:
        rho = S.min()
        eta = np.sum(np.exp(-1.0 / self.param_lambda * (S - rho)))
        return np.exp(-1.0 / self.param_lambda * (S - rho)) / eta

    def _moving_average_filter(self, xx: np.ndarray, window_size: int) -> np.ndarray:
        kernel = np.ones(window_size) / window_size
        return np.array([np.convolve(x, kernel, mode='same') for x in xx.T]).T


@jit(nopython=True)
def dynamics(state: np.ndarray, control: np.ndarray, m: float, xU: float, xUU: float, 
            yV: float, yVV: float, pos_mot_x: float, pos_mot_y: float, nR: float, nRR: float, 
            Iz: float, dt: float,a_v_max: float,a_w_max: float,v_max: float,w_max: float) -> np.ndarray:
    x, y, theta, u, v, r = state
    F_left, F_right, alpha_left, alpha_right = control
    alpha_left= alpha_right =0
    cos_alpha_left = np.cos(alpha_left)
    cos_alpha_right = np.cos(alpha_right)
    sin_alpha_left = np.sin(alpha_left)
    sin_alpha_right = np.sin(alpha_right)

    acc_u = (F_left * cos_alpha_left + F_right * cos_alpha_right - xU * u - xUU * u * abs(u)) / m
    acc_v = (F_left * sin_alpha_left + F_right * sin_alpha_right - yV * v - yVV * v * abs(v)) / m
    acc_r = ((-pos_mot_y * F_left * cos_alpha_left + pos_mot_y * F_right * cos_alpha_right) +
             (pos_mot_x * F_right * sin_alpha_right - pos_mot_x * F_left * sin_alpha_left) -
             nR * r - nRR * r * abs(r)) / Iz

    if acc_u > a_v_max:
        acc_u = a_v_max
    elif acc_u < -a_v_max:
        acc_u = -a_v_max

    if acc_v > a_v_max:
        acc_v = a_v_max
    elif acc_v < -a_v_max:
        acc_v = -a_v_max

    if acc_r> a_w_max:
        acc_r= a_w_max
    elif acc_r< -a_w_max:
        acc_r= -a_w_max

        

    u += acc_u * dt
    v += acc_v * dt
    r += acc_r * dt

    if u > v_max:
        u = v_max
    elif u < -v_max:
        u = -v_max

    if v > v_max:
        v = v_max
    elif v < -v_max:
        v = -v_max

    if r > w_max:
        r = w_max
    elif r < -w_max:
        r = -w_max

    theta += r * dt
    theta = (theta + np.pi) % (2 * np.pi) - np.pi

    x_vel = u * np.cos(theta) - v * np.sin(theta)
    y_vel = u * np.sin(theta) + v * np.cos(theta)

    x += x_vel * dt
    y += y_vel * dt

    return np.array([x, y, theta, u, v, r])


@jit(nopython=True)
def huber_loss(delta, delta_0=1.0) -> float:
    return delta**2 if abs(delta) <= delta_0 else 2 * delta_0 * abs(delta) - delta_0**2


@jit(nopython=True)
def stage_cost(state: np.ndarray, target: np.ndarray, stage_cost_weight: np.ndarray) -> float:
    x, y, theta, u, v, r = state
    x_t, y_t, theta_t = target
    theta_t = np.arctan2(-y, -x)  # Orientation tangentielle vers le but

    # Calcul du coût
    d_o = np.linalg.norm(np.array([x, y]) - np.array([0, 0]))
    return (stage_cost_weight[0] * huber_loss(35-x, 5.0) +
            stage_cost_weight[1] * huber_loss(15-y, 5.0) +
            stage_cost_weight[2] * (theta - theta_t)**2)


@jit(nopython=True)
def terminal_cost(state: np.ndarray, target: np.ndarray, stage_cost_weight: np.ndarray) -> float:
    return stage_cost(state, target, stage_cost_weight)

@jit(nopython=True)
def _g( v: np.ndarray,max_thrust,max_angle) -> np.ndarray:
    """Sature les commandes pour respecter les vitesses maximales"""
    # Saturation manuelle en utilisant des conditions
    if v[0] > max_thrust:
        v[0] = max_thrust
    elif v[0] < -max_thrust:
        v[0] = -max_thrust

    if v[1] > max_thrust:
        v[1] = max_thrust
    elif v[1] < -max_thrust:
        v[1] = -max_thrust

    if v[2] > max_angle:
        v[2] = max_angle
    elif v[2] < -max_angle:
        v[2] = -max_angle

    if v[3] > max_angle:
        v[3] = max_angle
    elif v[3] < -max_angle:
        v[3] = -max_angle
    return v
# Code de simulation inchangé
@jit(nopython=True, parallel=True)
def boucle(u,S,K,T,state_var,observed_state,epsilon,m, xU, xUU, yV, yVV, pos_mot_x, 
            pos_mot_y, nR, nRR, Iz, dt,max_thrust,max_angle,target_state,stage_cost_weight,a_v_max,a_w_max,v_max,w_max):
    for k in prange(K):
        v = np.zeros((T, state_var))
        x = observed_state

        for t in range(T):
            # Détermine si l'on exploite ou explore
            if k < 0.9 * K:
                v[t] = _g(u[t] + epsilon[k, t],max_thrust,max_angle)
            else:
                v[t] = _g(epsilon[k, t],max_thrust,max_angle)

            # Met à jour l'état avec le modèle du véhicule
            x = dynamics(x, v[t], m, xU, xUU, yV, yVV, pos_mot_x, pos_mot_y, nR, nRR, Iz, dt,a_v_max,a_w_max,v_max,w_max)

            # Calcul du coût pour cet état
            S[k] += stage_cost(x, target_state, stage_cost_weight)
        # Ajout du coût terminal
        S[k] += terminal_cost(x, target_state, stage_cost_weight)
    return S

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
    waypoints = generate_circular_waypoints(radius=10.0, num_points=500)

    # Configuration de la simulation
    horizon_step_T = 25
    sim_steps = 1000  # Nombre total d'étapes de simulation

    # Initialisation de l'état du véhicule et du contrôleur
    initial_state = np.array([30.0,0.0, 0.0, 0.0,0.0,0.0])  # Démarre au bas du cercle
    controller = MPPIControllerForAquabot(horizon_step_T=horizon_step_T)

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
            print("Point atteint ! ",current_waypoint_index,waypoints[current_waypoint_index])


        # Calculer le contrôle pour atteindre le waypoint actuel
        start_time = time.time()  # Temps avant l'exécution
        control, _ = controller.calc_control_input(current_state, target_state)
        control[2]=0.
        control[3]=0.
        current_state = dynamics(current_state, control, controller.m, controller.xU, controller.xUU, controller.yV, controller.yVV, 
                                controller.pos_mot_x, controller.pos_mot_y, controller.nR, controller.nRR, controller.Iz, 
                                controller.dt, controller.a_v_max, controller.a_w_max, controller.v_max, controller.w_max)
        end_time = time.time() 

        execution_time = end_time - start_time  # Calcul du temps écoulé
        print(f"Temps d'exécution : {execution_time} secondes")
        states.append(current_state)
        print(i)
        print(current_state)
        print(control)
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
    # Ajouter les flèches pour représenter l'orientation
    arrow_scale = 1.5  # Ajustez pour changer la taille des flèches
    u_vals = np.cos(theta_vals) * arrow_scale  # Composante x des flèches
    v_vals = np.sin(theta_vals) * arrow_scale  # Composante y des flèches

    # Afficher les flèches
    plt.quiver(x_vals, y_vals, u_vals, v_vals, angles='xy', scale_units='xy', scale=1, color="blue", label="Orientation")

    plt.xlabel("Position x (m)")
    plt.ylabel("Position y (m)")
    plt.title("Trajectoire du véhicule avec orientation")
    plt.legend()
    plt.grid()

    # Affichage des commandes
    plt.subplot(1, 2, 2)
    plt.plot(theta_vals, label="Vitesse linéaire (v)")
    # plt.plot(omega_controls, label="Vitesse angulaire (omega)")
    plt.xlabel("Étape de simulation")
    plt.ylabel("Commande")
    plt.title("Commandes de contrôle")
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.show(block=True)

if __name__ == "__main__":
    #   cProfile.run('run_simulation_mppi_differential_drive()')
    run_simulation_mppi_differential_drive()

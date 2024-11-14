import numpy as np
from numba import float64, int32
from numba.typed import Dict
from numba.experimental import jitclass
import time
from typing import Tuple  # Importation de Tuple

spec = [
    ('T', int32),
    ('K', int32),
    ('sigma', float64[:]),
    ('param_lambda', float64),
    ('stage_cost_weight', float64[:]),
    ('u_prev', float64[:,:]),
    ('v_max', float64),
    ('w_max', float64),
    ('a_v_max', float64),
    ('a_w_max', float64),
    ('dt', float64),
    ('state_var', int32),
    ('m', float64),
    ('Iz', float64),
    ('xU', float64),
    ('xUU', float64),
    ('yV', float64),
    ('yVV', float64),
    ('nR', float64),
    ('nRR', float64),
    ('max_thrust', float64),
    ('max_angle', float64),
    ('pos_mot_x', float64),
    ('pos_mot_y', float64)
]

@jitclass(spec)
class MPPIControllerForAquabot:
    def __init__(
            self,
            horizon_step_T: int = 25,
            number_of_samples_K: int = 600,
            sigma: np.ndarray = np.array([500, 500, 0.15, 0.15]),
            param_lambda: float = 1.0,
            stage_cost_weight: np.ndarray = np.array([0.009, 8.0, 30.0]),  # poids pour [x, y, theta]
            v_max: float = 1.0,  # Limite pour la vitesse linéaire maximale
            w_max: float = 1.0,  # Limite pour la vitesse angulaire maximale
            a_v_max: float = 500.0,  # Limite pour l'accélération linéaire maximale
            a_w_max: float = 0.8,  # Limite pour l'accélération angulaire maximale
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
        self.max_thrust = 4000
        self.max_angle = np.pi / 4
        self.pos_mot_x = -3
        self.pos_mot_y = 0.6

    def _g(self, v: np.ndarray) -> np.ndarray:
        """Sature les commandes pour respecter les vitesses maximales"""
        return v

    def calc_control_input(self, observed_state: np.ndarray, target_state: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        u = self.u_prev
        S = np.zeros(self.K)  # Coût total pour chaque échantillon
        epsilon = np.random.normal(0, self.sigma, (self.K, self.T, self.state_var))  # Bruit de contrôle

        # Boucle sur les échantillons avec parallélisation
        for k in range(self.K):
            v = np.zeros((self.T, self.state_var))
            x = observed_state

            for t in range(self.T):
                # Détermine si l'on exploite ou explore
                if k < 0.9 * self.K:
                    v[t] = self._g(u[t] + epsilon[k, t])
                else:
                    v[t] = self._g(epsilon[k, t])

                # Met à jour l'état avec le modèle du véhicule
                x = self.dynamics(x, v[t])

                # Calcul du coût pour cet état
                S[k] += self.stage_cost(x, target_state)

            # Ajout du coût terminal
            S[k] += self.terminal_cost(x, target_state)

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

        return self._g(u[0]), u

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

    def dynamics(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        x, y, theta, u, v, r = state
        F_left, F_right, alpha_left, alpha_right = control

        cos_alpha_left = np.cos(alpha_left)
        cos_alpha_right = np.cos(alpha_right)
        sin_alpha_left = np.sin(alpha_left)
        sin_alpha_right = np.sin(alpha_right)

        acc_u = (F_left * cos_alpha_left + F_right * cos_alpha_right - self.xU * u + self.xUU * u * abs(u)) / self.m
        acc_v = (F_left * sin_alpha_left + F_right * sin_alpha_right - self.yV * v + self.yVV * v * abs(v)) / self.m
        acc_r = ((-self.pos_mot_y * F_left * cos_alpha_left + self.pos_mot_y * F_right * cos_alpha_right) +
                 (self.pos_mot_x * F_right * sin_alpha_right - self.pos_mot_x * F_left * sin_alpha_left) -
                 self.nR * r - self.nRR * r * abs(r)) / self.Iz

        u += acc_u * self.dt
        v += acc_v * self.dt
        r += acc_r * self.dt

        theta += r * self.dt
        theta = (theta + np.pi) % (2 * np.pi) - np.pi

        x_vel = u * np.cos(theta) - v * np.sin(theta)
        y_vel = u * np.sin(theta) + v * np.cos(theta)

        x += x_vel * self.dt
        y += y_vel * self.dt

        return np.array([x, y, theta, u, v, r])

    def huber_loss(self, delta, delta_0=1.0) -> float:
        return delta**2 if abs(delta) <= delta_0 else 2 * delta_0 * abs(delta) - delta_0**2

    def stage_cost(self, state: np.ndarray, target: np.ndarray) -> float:
        x, y, theta, u, v, r = state
        target_x, target_y, target_theta = target
        d_o = np.linalg.norm([x - target_x, y - target_y])
        return (self.stage_cost_weight[0] * d_o + self.stage_cost_weight[1] * self.huber_loss(d_o, 1.0) + self.stage_cost_weight[2] * self.huber_loss(abs(theta - target_theta), 1.0))

    def terminal_cost(self, state: np.ndarray, target: np.ndarray) -> float:
        return 0.5 * np.linalg.norm(state[:2] - target[:2]) + 0.5 * abs(state[2] - target[2])


import numpy as np
from typing import Tuple
from py_control.mppi_pythran_normal import _g, dynamics,compute_lateral_error,stage_cost,boucle

#------------------------------------------------------------------------------------------------
#MPPI controller class
#------------------------------------------------------------------------------------------------

class MPPIControllerForAquabot:
    def __init__(
            self,
            horizon_step_T: int = 25,
            number_of_samples_K: int = 800,
            sigma: np.ndarray = np.array([60, 60, 0.1, 0.1]),
            param_lambda: float = 1.,
            stage_cost_weight: np.ndarray = np.array([45, 45, 300.0,0.0]),  # poids pour [x, y, theta]
            control_var=4,
            dt=0.1,
            max_thrust=1000.0
    ):
        self.T = horizon_step_T
        self.K = number_of_samples_K
        self.sigma = sigma
        self.param_lambda = param_lambda
        self.stage_cost_weight = stage_cost_weight
        self.u_prev = np.zeros((self.T, control_var))  # initialisation des commandes précédentes [v, omega]
        self.dt = dt

        
        self.control_var = control_var

        self.m = 1130.0
        self.Iz = 446.0
        self.xU = 182.0
        self.xUU = 224.0
        self.yV = 183.0
        self.yVV = 149.0
        self.nR = 1199.0
        self.nRR = 979.0

        self.max_thrust = max_thrust
        self.max_angle = np.pi / 4
        self.pos_mot_x = 3.2
        self.pos_mot_y = 0.6



    
    def calc_control_input(self, observed_state: np.ndarray, target_state: np.ndarray, mode :int,plan_array: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        
        if self.u_prev.shape[0] > self.T:
            self.u_prev = self.u_prev[:self.T, :]
    
        # Si self.u_prev a trop peu de lignes, on complète avec les dernières valeurs
        elif self.u_prev.shape[0] < self.T:
            missing_rows = self.T - self.u_prev.shape[0]
            last_values = self.u_prev[-1:, :]  # Dernière ligne
            # Répète la dernière ligne pour les lignes manquantes
            fill_values = np.tile(last_values, (missing_rows, 1))
            self.u_prev = np.vstack((self.u_prev, fill_values))
        u = self.u_prev

        
        S = np.zeros(self.K)  # Coût total pour chaque échantillon
        epsilon = np.random.normal(0, self.sigma, (self.K, self.T, self.control_var))  # Bruit de contrôle

        # Boucle sur les échantillons
         
        S=boucle(u,S,self.K,self.T,self.control_var,observed_state,epsilon,self.m, self.xU, self.xUU, self.yV, self.yVV, 
                self.pos_mot_x, self.pos_mot_y, self.nR, self.nRR, self.Iz, self.dt,self.max_thrust,
                self.max_angle,target_state,self.stage_cost_weight, mode,plan_array)

           

        # Calcul des poids basés sur les coûts
        w = self._compute_weights(S)

        # Calcul du contrôle pondéré par les poids
        w_epsilon = np.sum(w[:, None, None] * epsilon, axis=0)

        # Appliquer le filtre de moyenne mobile
        w_epsilon = self._moving_average_filter(xx=w_epsilon, window_size=25)

        # Mise à jour des commandes avec les valeurs lissées
        u = u + w_epsilon

        # Mise à jour de u_prev
        self.u_prev[:-1] = u[1:]  # Décale les commandes vers la gauche
        self.u_prev[-1] = u[-1]

        return _g(u[0],self.max_thrust,self.max_angle), u


    def _compute_weights(self, S: np.ndarray) -> np.ndarray:
        rho = S.min()
        eta = np.sum(np.exp(-1.0 / self.param_lambda * (S - rho)))
        return np.exp(-1.0 / self.param_lambda * (S - rho)) / eta

    def _moving_average_filter(self, xx: np.ndarray, window_size: int) -> np.ndarray:
        kernel = np.ones(window_size) / window_size
        return np.array([np.convolve(x, kernel, mode='same') for x in xx.T]).T

#------------------------------------------------------------------------------------------------
#Keeped for now for legacy and debug, these funtions are compiled aot and not jot now.
#------------------------------------------------------------------------------------------------

#------------------------------------------------------------------------------------------------
#Function that needed to be compiled and parallelized, outside of the class because numba doesn't handle 
#complex objects like self
#------------------------------------------------------------------------------------------------


# #calculte the next state of the robot thanks to the dynamics' equation
# @njit(nopython=True, cache=True)
# def dynamics(state: np.ndarray, control: np.ndarray, m: float, xU: float, xUU: float, 
#             yV: float, yVV: float, pos_mot_x: float, pos_mot_y: float, nR: float, nRR: float, 
#             Iz: float, dt: float) -> np.ndarray:
    
#     x, y, theta, u, v, r = state
#     F_left, F_right, alpha_left, alpha_right = control
#     cos_alpha_left = np.cos(alpha_left)
#     cos_alpha_right = np.cos(alpha_right)
#     sin_alpha_left = np.sin(alpha_left)
#     sin_alpha_right = np.sin(alpha_right)

#     #Newton's second law, input + hydro + coriolis 
#     acc_u = (F_left * cos_alpha_left + F_right * cos_alpha_right - xU * u - xUU * u * abs(u)+ m*v*r) / m
#     acc_v = (F_left * sin_alpha_left + F_right * sin_alpha_right - yV * v - yVV * v * abs(v)- m*u*r) / m
#     acc_r = ((-pos_mot_y * F_left * cos_alpha_left + pos_mot_y * F_right * cos_alpha_right) +
#              ( - pos_mot_x * F_left * sin_alpha_left - pos_mot_x * F_right * sin_alpha_right) -
#              nR * r - nRR * r * abs(r)) / Iz

#     u += acc_u * dt
#     v += acc_v * dt
#     r += acc_r * dt


#     theta += r * dt
#     theta = (theta + np.pi) % (2 * np.pi) - np.pi
    
#     #in world coordinates
#     x_vel = u * np.cos(theta) - v * np.sin(theta)
#     y_vel = u * np.sin(theta) + v * np.cos(theta)

#     x += x_vel * dt
#     y += y_vel * dt

#     return np.array([x, y, theta, u, v, r])


# # function to compute the cost of the calculated state
# @njit(nopython=True, cache=True)
# def stage_cost(state: np.ndarray, target: np.ndarray, stage_cost_weight: np.ndarray,mode :int,plan_array) -> float:
#     x, y, theta, u, v, r = state

#     if mode == 0 : #follow point
#         x_t, y_t, theta_t = target
#         diff_theta = theta-theta_t
#         diff_theta = (diff_theta + np.pi) % (2 * np.pi) - np.pi
#         return (stage_cost_weight[0] *(x_t-x)**2 +
#                 stage_cost_weight[1] * (y_t-y)**2 +
#                 stage_cost_weight[2] * diff_theta**2)
   

#     elif mode == 1 : #follow path
#         x_t, y_t, theta_t = target
#         e_lat = compute_lateral_error(state, plan_array)
#         diff_theta = theta-theta_t
#         diff_theta = (diff_theta + np.pi) % (2 * np.pi) - np.pi
#         return (stage_cost_weight[0] *e_lat**2
#                 +stage_cost_weight[1] *diff_theta**2
#                 +(stage_cost_weight[2]-4*abs(diff_theta))*(9.5-u)**2)

#     elif mode == 2 : #circle for qr_code
#         x_t, y_t, _ = target 
#         d_o = np.linalg.norm(np.array([x, y]) - np.array([x_t, y_t]))
#         theta_t = np.arctan2(y-y_t, x_t-x)
#         diff_theta = theta-theta_t
#         diff_theta = (diff_theta + np.pi) % (2 * np.pi) - np.pi
#         return (stage_cost_weight[0] * (3-u)**2 +
#                 stage_cost_weight[1] * (d_o-5)**2  +
#                 stage_cost_weight[2] * (3/10-v)**2) 
    
#     elif mode == 3 : #for bonus phase
#         x_t, y_t, _ = target 
#         d_o = np.linalg.norm(np.array([x, y]) - np.array([x_t, y_t]))
#         theta_t = np.arctan2(y_t-y, x_t-x)
#         diff_theta = theta-theta_t
#         diff_theta = (diff_theta + np.pi) % (2 * np.pi) - np.pi
#         return (stage_cost_weight[0] * (1.0-v)**2 + 
#                 stage_cost_weight[1] * (d_o-10.0)**2  +
#                 stage_cost_weight[2] * diff_theta**2) 

    

# #to emphasize on the last state
# @njit(nopython=True, cache=True)
# def terminal_cost(state: np.ndarray, target: np.ndarray, stage_cost_weight: np.ndarray, mode :int,plan_array) -> float:
#     return 15 * stage_cost(state, target, stage_cost_weight,mode,plan_array)

# #to adp, we don't use np function as they are not supported by numba
# @njit(nopython=True, cache=True)
# def _g( v: np.ndarray,max_thrust,max_angle) -> np.ndarray:
#     if v[0] > max_thrust:
#         v[0] = max_thrust
#     elif v[0] < -max_thrust:
#         v[0] = -max_thrust

#     if v[1] > max_thrust:
#         v[1] = max_thrust
#     elif v[1] < -max_thrust:
#         v[1] = -max_thrust

#     if v[2] > max_angle:
#         v[2] = max_angle
#     elif v[2] < -max_angle:
#         v[2] = -max_angle

#     if v[3] > max_angle:
#         v[3] = max_angle
#     elif v[3] < -max_angle:
#         v[3] = -max_angle
#     return v

# #main part of the MPPI algorithm, compute the total for the T horizon steps K times
# @njit(parallel=True, cache=True)
# def boucle(u,S,K,T,control_var,observed_state,epsilon,m, xU, xUU, yV, yVV, pos_mot_x, 
#             pos_mot_y, nR, nRR, Iz, dt,max_thrust,max_angle,target_state,stage_cost_weight, mode,plan_array):
#     for k in prange(K):
#         v = np.zeros((T, control_var))
#         x = observed_state

#         for t in range(T):
#             # Use the precedent values or explore ? Made for avoiding the local minimum
#             if k < 0.9 * K:
#                 v[t] = _g(u[t] + epsilon[k, t],max_thrust,max_angle)
#             else:
#                 v[t] = _g(epsilon[k, t],max_thrust,max_angle)

#             # Calculate theoritical next step
#             x = dynamics(x, v[t], m, xU, xUU, yV, yVV, pos_mot_x, pos_mot_y, nR, nRR, Iz, dt)

#             # Cost computation
#             S[k] += stage_cost(x, target_state, stage_cost_weight, mode,plan_array)
#         # Terminal cost
#         S[k] += 15*stage_cost(x, target_state, stage_cost_weight, mode,plan_array)
#     return S

# #to find the perpendicular distance to the path during path following
# @njit(nopython=True, cache=True)
# def compute_lateral_error(state: np.ndarray, path: np.ndarray) -> float:
#     x, y = state[:2]  # Les coordonnées actuelles de l'USV
#     min_dist = float('inf')  # Initialisation de la distance minimale à l'infini
#     e_lat = 0.0  # Initialisation de l'erreur latérale

#     # Parcours de tous les segments du chemin (chaque paire de points successifs)
#     for i in range(len(path) - 1):
#         # Points du segment (point i et point i+1 du tableau de positions)
#         p1 = path[i]  # Coordonnée du premier point du segment [x1, y1]
#         p2 = path[i + 1]  # Coordonnée du deuxième point du segment [x2, y2]

#         # Vecteur du segment [p1, p2]
#         segment = np.asarray(p2 - p1, dtype=np.float64)
#         # Vecteur entre l'USV (x, y) et p1
#         point_to_p1 = np.asarray([x, y], dtype=np.float64) - p1

#         # Calcul du produit scalaire entre le vecteur du segment et le vecteur point-to-p1
#         segment_length_squared = np.dot(segment, segment)  # |segment|^2

#         # Calcul de la projection du point (x, y) sur le segment
#         t = np.dot(point_to_p1, segment) / segment_length_squared

#         # Clamping de t pour s'assurer que la projection est bien sur le segment
#         if t < 0.0:
#             t = 0.0
#         elif t > 1.0:
#             t = 1.0

#         # Calcul de la projection sur le segment
#         proj = p1 + t * segment

#         # Calcul de la distance entre l'USV et la projection sur le segment
#         dist = np.linalg.norm(np.array([x, y]) - proj)

#         # Mise à jour de la distance minimale et de l'erreur latérale
#         if dist < min_dist:
#             min_dist = dist
#             e_lat = dist

#     return e_lat


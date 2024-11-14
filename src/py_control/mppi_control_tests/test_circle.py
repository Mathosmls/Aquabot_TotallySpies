import numpy as np
from scipy.optimize import minimize

# Données et paramètres
m = 1130.0
Iz = 456.0
xU = 182.0
xUU = 224.0
yV = 183.0
yVV = 149.0
nR = 1199.0
nRR = 979.0
max_thrust = 4000
max_angle = np.pi / 4
pos_mot_x = -3.
pos_mot_y = 0.6
import numpy as np
from scipy.optimize import minimize
dt = 0.1       # Pas de temps

# Vitesses actuelles et désirées
u, v, r = 0.0, 0.0, -0.0   # Vitesses actuelles
u_desired, v_desired, r_desired = 1.0, 0.5, -0.2  # Vitesses cibles

# Calcul des accélérations cibles pour atteindre les vitesses désirées
acc_u_target = (u_desired - u) / dt
acc_v_target = (v_desired - v) / dt
acc_r_target = (r_desired - r) / dt

# Fonction à minimiser : erreur entre les accélérations obtenues et les accélérations cibles
def objective(vars):
    F_left, F_right, alpha_left, alpha_right = vars
    cos_alpha_left = np.cos(alpha_left)
    cos_alpha_right = np.cos(alpha_right)
    sin_alpha_left = np.sin(alpha_left)
    sin_alpha_right = np.sin(alpha_right)
    
    # Calcul des accélérations
    acc_u = (F_left * cos_alpha_left + F_right * cos_alpha_right - xU * u - xUU * u * abs(u) + m * v * r) / m
    acc_v = (F_left * sin_alpha_left + F_right * sin_alpha_right - yV * v - yVV * v * abs(v) - m * u * r) / m
    acc_r = ((-pos_mot_y * F_left * cos_alpha_left + pos_mot_y * F_right * cos_alpha_right) +
             (pos_mot_x * F_right * sin_alpha_right - pos_mot_x * F_left * sin_alpha_left) -
             nR * r - nRR * r * abs(r)) / Iz
    
    # Calcul de l'erreur quadratique entre les accélérations obtenues et les cibles
    error_u = (acc_u - acc_u_target) ** 2
    error_v = (acc_v - acc_v_target) ** 2
    error_r = (acc_r - acc_r_target) ** 2
    return error_u + error_v + error_r

# Initial guess for the variables: F_left, F_right, alpha_left, alpha_right
initial_guess = [1.0, 1.0, 0.0, 0.0]  # Forces de 1 N et angles de 0 radians

# Définition des bornes pour les forces et angles
bounds = [(-1000, 1000), (-1000, 1000), (-np.pi/4, np.pi/4), (-np.pi/4, np.pi/4)]

# Résolution de l'optimisation
result = minimize(objective, initial_guess, bounds=bounds)

# Résultats
if result.success:
    F_left_opt, F_right_opt, alpha_left_opt, alpha_right_opt = result.x
    print(f"Optimized values:\n F_left: {F_left_opt}\n F_right: {F_right_opt}\n alpha_left: {alpha_left_opt}\n alpha_right: {alpha_right_opt}")
else:
    print("Optimization did not converge.")

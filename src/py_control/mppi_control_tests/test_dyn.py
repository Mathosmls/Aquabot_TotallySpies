import numpy as np
from scipy.integrate import solve_ivp

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
v_max = 7.0
w_max = 1000
dt = 0.01
a_v_max = 1000 * dt  # Nouvelle limite d'accélération linéaire
a_w_max = 1000 * dt  # Nouvelle limite d'accélération angulaire
import matplotlib.pyplot as plt


def system(t,state, control):
    x, y, theta,u,v,r = state
    
    # Obtenir les forces et les angles pour ce temps t
    F_left, F_right, alpha_left, alpha_right = control
    # Calcul des accélérations
    alpha_left=-alpha_left
    alpha_right=-alpha_right
    acc_u = (F_left * np.cos(alpha_left) + F_right * np.cos(alpha_right) - xU * u - xUU * u * abs(u)) / m
    acc_v = (F_left * np.sin(alpha_left) + F_right * np.sin(alpha_right) - yV * v - yVV * v * abs(v)) / m
    acc_r = ((-pos_mot_y * F_left * np.cos(alpha_left) + pos_mot_y * F_right * np.cos(alpha_right)) +
             (pos_mot_x * F_right * np.sin(alpha_right) - pos_mot_x * F_left * np.sin(alpha_left)) -
             nR * r - nRR * r * abs(r)) / Iz

    # Calcul des dérivées (vitesse et position)


    x_vel = u * np.cos(theta) - v * np.sin(theta)
    y_vel = u * np.sin(theta) + v * np.cos(theta)



    return [x_vel, y_vel, r, acc_u, acc_v, acc_r]

def dynamics(state: np.ndarray, control: np.ndarray, m: float, xU: float, xUU: float, 
            yV: float, yVV: float, pos_mot_x: float, pos_mot_y: float, nR: float, nRR: float, 
            Iz: float, dt: float,a_v_max: float,a_w_max: float,v_max: float,w_max: float) -> np.ndarray:
    x, y, theta, u, v, r = state
    F_left, F_right, alpha_left, alpha_right = control
    F_left= 63.59923686702797
    F_right= 86.27180545146147
    alpha_left= -0.2471395076094389
    alpha_right= 0.147648014679554
    theta = (theta + np.pi) % (2 * np.pi) - np.pi
    cos_alpha_left = np.cos(-alpha_left)
    cos_alpha_right = np.cos(alpha_right)
    sin_alpha_left = np.sin(-alpha_left)
    sin_alpha_right = np.sin(alpha_right)
    # print(F_left * cos_alpha_left , F_right * cos_alpha_right,F_left * sin_alpha_left,F_right * sin_alpha_right)
    acc_u = (F_left * cos_alpha_left + F_right * cos_alpha_right - xU * u - xUU * u * abs(u)+ m*v*r) / m 
    acc_v = (F_left * sin_alpha_left + F_right * sin_alpha_right - yV * v - yVV * v * abs(v)- m*u*r) / m 
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
    # u = 0.85
    # v=0.72
    # r=-0.224
    theta += r * dt
    theta = (theta + np.pi) % (2 * np.pi) - np.pi

    x_vel = u * np.cos(theta) - v * np.sin(theta)
    y_vel = u * np.sin(theta) + v * np.cos(theta)

    x += x_vel * dt
    y += y_vel * dt

    return np.array([x, y, theta, u, v, r])



t=50
state=np.array([30,0,0,0,0,0])
control=np.array([0,500,-np.pi/4,0])




# Accès aux résultats
T = 30  # durée totale de la simulation
t_span = (0, T)
dt = 0.1
t_eval = np.arange(0, T, dt)


# for i in t_eval :
#         sol = solve_ivp(system, [t, t + dt], state, args=(control,), method='RK45', rtol=1e-5, atol=1e-8)
#         state = sol.y[:, -1]
#         print(state)
states=[]
for i in range(1500) :
    state=  dynamics(state, control, m, xU, xUU, 
            yV, yVV, pos_mot_x, pos_mot_y, nR, nRR, 
            Iz, dt,a_v_max,a_w_max,v_max,w_max) 
    print(state)
    states.append(state)
control=np.array([0,0,0,0])
print("stop")
x_vals = [state[0] for state in states]
y_vals = [state[1] for state in states]
theta_vals = [state[2] for state in states]

arrow_scale = 1.5  # Ajustez pour changer la taille des flèches
u_vals = np.cos(theta_vals) * arrow_scale  # Composante x des flèches
v_vals = np.sin(theta_vals) * arrow_scale  # Composante y des flèches

# Afficher les flèches
plt.plot(x_vals, y_vals, label="Trajectoire")
plt.quiver(x_vals, y_vals, u_vals, v_vals, angles='xy', scale_units='xy', scale=1, color="blue", label="Orientation")
plt.show(block=True)


# t=200
# for i in range(t) :
#     state=  dynamics(state, control, m, xU, xUU, 
#             yV, yVV, pos_mot_x, pos_mot_y, nR, nRR, 
#             Iz, dt,a_v_max,a_w_max,v_max,w_max) 
#     print(state)
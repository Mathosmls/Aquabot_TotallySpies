import numpy as np

# Fonction _g
#pythran export _g(float64[:],float,float)
def _g(v: np.ndarray, max_thrust: float, max_angle: float) -> np.ndarray:
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



#pythran export dynamics(float64[:],float64[:],float, float, float, float, float, float, float, float, float, float, float)
def dynamics(state, control, m, xU, xUU, yV, yVV, pos_mot_x, pos_mot_y, nR, nRR, Iz, dt):
    x, y, theta, u, v, r = state
    F_left, F_right, alpha_left, alpha_right = control
    cos_alpha_left = np.cos(alpha_left)
    cos_alpha_right = np.cos(alpha_right)
    sin_alpha_left = np.sin(alpha_left)
    sin_alpha_right = np.sin(alpha_right)

    acc_u = (F_left * cos_alpha_left + F_right * cos_alpha_right - xU * u - xUU * u * abs(u) + m * v * r) / m
    acc_v = (F_left * sin_alpha_left + F_right * sin_alpha_right - yV * v - yVV * v * abs(v) - m * u * r) / m
    acc_r = ((-pos_mot_y * F_left * cos_alpha_left + pos_mot_y * F_right * cos_alpha_right) +
             (-pos_mot_x * F_left * sin_alpha_left - pos_mot_x * F_right * sin_alpha_right) -
             nR * r - nRR * r * abs(r)) / Iz

    u += acc_u * dt
    v += acc_v * dt
    r += acc_r * dt

    theta += r * dt
    theta = (theta + np.pi) % (2 * np.pi) - np.pi

    x_vel = u * np.cos(theta) - v * np.sin(theta)
    y_vel = u * np.sin(theta) + v * np.cos(theta)

    x += x_vel * dt
    y += y_vel * dt

    return np.array([x, y, theta, u, v, r])


#pythran export compute_lateral_error(float64[:],float64[:,:])
def compute_lateral_error(state, path):
    x, y = state[:2]
    min_dist = float('inf')
    e_lat = 0.0

    for i in range(len(path) - 1):
        p1 = path[i]
        p2 = path[i + 1]
        segment = np.asarray(p2 - p1, dtype=np.float64)
        point_to_p1 = np.asarray([x, y], dtype=np.float64) - p1
        segment_length_squared = np.dot(segment, segment)
        t = np.dot(point_to_p1, segment) / segment_length_squared
        t = max(0.0, min(1.0, t))
        proj = p1 + t * segment
        dist = np.linalg.norm(np.array([x, y]) - proj)

        if dist < min_dist:
            min_dist = dist
            e_lat = dist

    return e_lat


#pythran export stage_cost(float64[:],float64[:],float64[:],int,float64[:,:])
def stage_cost(state, target, stage_cost_weight, mode, plan_array):
    x, y, theta, u, v, r = state

    if mode == 1:  # Follow path
        x_t, y_t, theta_t = target
        e_lat = compute_lateral_error(state, plan_array)
        diff_theta = theta - theta_t
        diff_theta = (diff_theta + np.pi) % (2 * np.pi) - np.pi
        return (stage_cost_weight[0] * e_lat ** 2 +
                stage_cost_weight[1] * diff_theta ** 2 +
                (stage_cost_weight[2] - 4 * abs(diff_theta)) * (9.5 - u) ** 2)

    elif mode == 2:  # Circle for QR code
        x_t, y_t, distance = target
        d_o = np.linalg.norm(np.array([x, y]) - np.array([x_t, y_t]))
        theta_t = np.arctan2(y - y_t, x_t - x)
        diff_theta = theta - theta_t
        diff_theta = (diff_theta + np.pi) % (2 * np.pi) - np.pi
        return (stage_cost_weight[0] * (3 - u) ** 2 +
                stage_cost_weight[1] * (d_o - distance) ** 2 +
                stage_cost_weight[2] * (u / distance - v) ** 2)

    elif mode == 3:  # Bonus phase
        x_t, y_t, speed = target
        d_o = np.linalg.norm(np.array([x, y]) - np.array([x_t, y_t]))
        theta_t = np.arctan2(y_t - y, x_t - x)
        diff_theta = theta - theta_t
        diff_theta = (diff_theta + np.pi) % (2 * np.pi) - np.pi
        return (stage_cost_weight[0] * (speed - v) ** 2 +
                stage_cost_weight[1] * (d_o - 10.0) ** 2 +
                stage_cost_weight[2] * diff_theta ** 2)

    else :  # mode 0, Follow point
        x_t, y_t, theta_t = target
        diff_theta = theta - theta_t
        diff_theta = (diff_theta + np.pi) % (2 * np.pi) - np.pi
        return (stage_cost_weight[0] * (x_t - x) ** 2 +
                stage_cost_weight[1] * (y_t - y) ** 2 +
                stage_cost_weight[2] * diff_theta ** 2)
    


#pythran export boucle(float64[:, :], float64[:], int, int, int, float64[:], float64[:, :, :], float, float, float, float, float, float, float, float, float, float, float, float, float, float64[:], float64[:], int, float64[:, :],float,float)
def boucle(u, S, K, T, control_var, observed_state, epsilon, m, xU, xUU, yV, yVV, pos_mot_x, pos_mot_y, nR, nRR, Iz, dt, max_thrust, max_angle, target_state, stage_cost_weight, mode, plan_array,terminal_cost_mult,explor):
    for k in range(K):
        v = np.zeros((T, control_var))
        x = observed_state
        for t in range(T):
            if k < explor * K:
                temp = np.asarray(u[t] + epsilon[k, t], dtype=np.float64)
                v[t]=np.asarray(_g(temp, max_thrust, max_angle))
            else:
                temp = np.asarray(epsilon[k, t], dtype=np.float64)
                v[t] = np.asarray(_g(temp, max_thrust, max_angle))

            x = dynamics(x, v[t], m, xU, xUU, yV, yVV, pos_mot_x, pos_mot_y, nR, nRR, Iz, dt)
            S[k] += stage_cost(x, target_state, stage_cost_weight, mode, plan_array)

        S[k] += terminal_cost_mult*stage_cost(x, target_state, stage_cost_weight, mode, plan_array)
    return S


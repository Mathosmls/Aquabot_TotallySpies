import numpy as np

def calculate_wind_velocity_and_direction(acceleration, velocity_robot, mass, coeff_x, coeff_y):
    """
    Calcule la vitesse et la direction du vent à partir de l'accélération et de la vitesse du robot.
    
    :param acceleration: Tuple (ax, ay) représentant l'accélération du robot en m/s^2.
    :param velocity_robot: Tuple (vx, vy) représentant la vitesse du robot en m/s.
    :param mass: Masse du robot en kg.
    :param coeff_x: Coefficient aérodynamique pour la composante X.
    :param coeff_y: Coefficient aérodynamique pour la composante Y.
    :return: Tuple (wind_speed, wind_direction) représentant la vitesse (m/s) et la direction du vent (degrés).
    """
    ax, ay = acceleration
    vx, vy = velocity_robot
    
    # Calcul des forces appliquées par le vent
    F_wind_x = mass * ax
    F_wind_y = mass * ay
    
    # Calcul des vitesses relatives (v_rel = sqrt(F / coeff))
    V_rel_x = np.sign(F_wind_x) * np.sqrt(abs(F_wind_x) / coeff_x)
    V_rel_y = np.sign(F_wind_y) * np.sqrt(abs(F_wind_y) / coeff_y)
    
    # Calcul de la vitesse du vent (v_wind = v_rel + v_robot)
    V_wind_x = V_rel_x + vx
    V_wind_y = V_rel_y + vy
    
    # Calcul de la vitesse et direction du vent
    wind_speed = np.sqrt(V_wind_x**2 + V_wind_y**2)  # Magnitude de la vitesse
    wind_direction = np.degrees(np.arctan2(V_wind_y, V_wind_x))  # Angle en degrés
    
    return wind_speed, wind_direction


# Exemple d'utilisation
if __name__ == "__main__":
    # Données d'entrée
    acceleration = (0.3, -0.1)  # Accélération du robot en m/s^2
    velocity_robot = (2, 1)     # Vitesse du robot en m/s
    mass = 10                   # Masse du robot en kg
    coeff_x = 0.5               # Coefficient aérodynamique X
    coeff_y = 0.3               # Coefficient aérodynamique Y
    
    # Calcul de la vitesse et de la direction du vent
    wind_speed, wind_direction = calculate_wind_velocity_and_direction(acceleration, velocity_robot, mass, coeff_x, coeff_y)
    
    # Affichage des résultats
    print(f"Vitesse du vent : {wind_speed:.2f} m/s")
    print(f"Direction du vent : {wind_direction:.2f} degrés (par rapport à l'axe X positif)")

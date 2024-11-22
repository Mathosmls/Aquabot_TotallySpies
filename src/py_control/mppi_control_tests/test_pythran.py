import time
from mppi_pythran_normal import _g, dynamics,compute_lateral_error,stage_cost,boucle
import numpy as np

x=np.array([5.0,7.0,5.0,5.0])
y=_g(x,5.0,2.0)
print(y)

def test_dynamics():
    # Initialize the state [x, y, theta, u, v, r]
    state = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.1], dtype=np.float64)

    # Initialize the control inputs [F_left, F_right, alpha_left, alpha_right]
    control = np.array([50.0, 50.0, np.pi / 6, -np.pi / 6], dtype=np.float64)

    # Parameters
    m = 500.0       # Mass (kg)
    xU = 50.0       # Drag coefficient in x
    xUU = 10.0      # Quadratic drag in x
    yV = 30.0       # Drag coefficient in y
    yVV = 15.0      # Quadratic drag in y
    pos_mot_x = 1.0 # Motor position x
    pos_mot_y = 1.0 # Motor position y
    nR = 5.0        # Rotational drag
    nRR = 2.0       # Quadratic rotational drag
    Iz = 1000.0     # Moment of inertia
    dt = 0.1        # Time step

    # Call the dynamics function
    new_state = dynamics(state, control, m, xU, xUU, yV, yVV, pos_mot_x, pos_mot_y, nR, nRR, Iz, dt)

    # Output results
    print("Initial State:", state)
    print("Control Inputs:", control)
    print("New State:", new_state)

def test_compute_lateral_error():
    # State [x, y, other_values...]
    state = np.array([3.0, 2.0, 0.0], dtype=np.float64)

    # Path: A set of 2D points forming a trajectory
    path = np.array([
        [0.0, 0.0],
        [5.0, 0.0],
        [0.0, 0.0],
        [5.0, 0.0],
        [0.0, 0.0],
        [5.0, 0.0],
        [0.0, 0.0],
        [5.0, 0.0],
        [0.0, 0.0],
        [5.0, 0.0],
        [0.0, 0.0],
        [5.0, 0.0],
        [0.0, 0.0],
        [5.0, 0.0],
        [0.0, 0.0],
        [5.0, 0.0],
        [10.0, 5.0]
    ], dtype=np.float64)

    # Call the function
    e_lat = compute_lateral_error(state, path)

    # Output results
    print("State:", state[:2])
    print("Path:\n", path)
    print("Lateral Error:", e_lat)


def test_stage_cost():
    # Example input values
    state = np.array([2.0, 3.0, 0.5, 1.0, 0.2, 0.1], dtype=np.float64)
    target = np.array([10.0, 10.0, 0.0], dtype=np.float64)
    stage_cost_weight = np.array([1.0, 1.0, 1.0], dtype=np.float64)
    mode = 1  # Path-following mode
    plan_array = np.array([
        [0.0, 0.0],
        [5.0, 0.0],
        [10.0, 5.0],
    ], dtype=np.float64)

    # Call the function
    cost = stage_cost(state, target, stage_cost_weight, mode, plan_array)

    # Output results
    print("State:", state)
    print("Target:", target)
    print("Stage Cost Weight:", stage_cost_weight)
    print("Mode:", mode)
    print("Plan Array:\n", plan_array)
    print("Computed Stage Cost:", cost)

def test_boucle() :

    # Initialize test inputs
    K = 1000  # Number of iterations
    T = 50   # Number of time steps
    control_var = 4  # Control variables
    observed_state = np.array([0.0, 0.0, 0.0, 1.0, 0.5, 0.1])  # Example state
    u = np.random.rand(T, control_var)  # Random controls
    S = np.zeros(K)  # Cost accumulator
    epsilon = np.random.rand(K, T, control_var) * 0.1  # Random noise
    m, xU, xUU, yV, yVV = 1.0, 0.1, 0.05, 0.1, 0.05  # Dynamics parameters
    pos_mot_x, pos_mot_y = 0.2, 0.2
    nR, nRR, Iz, dt = 0.05, 0.01, 0.1, 0.1
    max_thrust, max_angle = 10.0, np.pi / 4
    target_state = np.array([5.0, 5.0, 0.0,5.0])
    stage_cost_weight = np.array([1.0, 1.0, 1.0])
    mode = 1  # Path-following mode
    plan_array = np.array([[0.0, 0.0], [10.0, 0.0]])  # Straight-line path

    # Call the function
    output = boucle(u, S, K, T, control_var, observed_state, epsilon, m, xU, xUU, yV, yVV, 
                    pos_mot_x, pos_mot_y, nR, nRR, Iz, dt, max_thrust, max_angle, 
                    target_state, stage_cost_weight, mode, plan_array)

    # Verify the result
    print("Output costs:", output)


test_dynamics()
test_compute_lateral_error()
test_stage_cost()
start_time = time.time()
test_boucle()
end_time = time.time()
execution_time = end_time - start_time
print(execution_time)

#0.3531 en normal
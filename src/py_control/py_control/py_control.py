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
            horizon_step_T: int = 90,
            number_of_samples_K: int = 1200,
            sigma: np.ndarray = np.array([60, 60, 0.1, 0.1]),
            param_lambda: float = 1.,
            # stage_cost_weight: np.ndarray = np.array([25.0000, 10.0, 100.0]),  # poids pour [x, y, theta]
            stage_cost_weight: np.ndarray = np.array([45, 45., 300.0]),  # poids pour [x, y, theta]
            v_max: float = 500.0,  # Limite pour la vitesse linéaire maximale
            w_max: float = 100.3,  # Limite pour la vitesse angulaire maximale
            a_v_max: float = 500,  # Limite pour l'accélération linéaire maximale
            a_w_max: float = 500,  # Limite pour l'accélération angulaire maximale
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

        self.m = 1130.0
        self.Iz = 446.0
        self.xU = 182.0
        self.xUU = 224.0
        self.yV = 183.0
        self.yVV = 149.0
        self.nR = 1199.0
        self.nRR = 979.0

        self.max_thrust = 3000
        self.max_angle = np.pi / 4-0.01
        self.pos_mot_x = 3.0
        self.pos_mot_y = 0.6



    
    def calc_control_input(self, observed_state: np.ndarray, target_state: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        u = self.u_prev
        S = np.zeros(self.K)  # Coût total pour chaque échantillon
        epsilon = np.random.normal(0, self.sigma, (self.K, self.T, self.state_var))  # Bruit de contrôle
        # d_o = np.linalg.norm(np.array([observed_state[0], observed_state[1]]) - np.array([30, -15]))
        # if d_o < 4 :
        #     self.max_thrust = 200
        #     self.sigma: np.ndarray = np.array([20, 20, 0.1, 0.1])
        # else :
        #     self.max_thrust = 500
        #     self.sigma: np.ndarray = np.array([35, 35, 0.1, 0.1])

        # Boucle sur les échantillons
         
        S=boucle(u,S,self.K,self.T,self.state_var,observed_state,epsilon,self.m, self.xU, self.xUU, self.yV, self.yVV, 
                self.pos_mot_x, self.pos_mot_y, self.nR, self.nRR, self.Iz, self.dt,self.max_thrust,
                self.max_angle,target_state,self.stage_cost_weight,self.a_v_max,self.a_w_max,self.v_max,self.w_max)

           

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
    # theta = (theta + np.pi) % (2 * np.pi) - np.pi
    # alpha_left=alpha_right=0.
    cos_alpha_left = np.cos(alpha_left)
    cos_alpha_right = np.cos(alpha_right)
    sin_alpha_left = np.sin(alpha_left)
    sin_alpha_right = np.sin(alpha_right)


    acc_u = (F_left * cos_alpha_left + F_right * cos_alpha_right - xU * u - xUU * u * abs(u)+ m*v*r) / m
    acc_v = (F_left * sin_alpha_left + F_right * sin_alpha_right - yV * v - yVV * v * abs(v)- m*u*r) / m
    acc_r = ((-pos_mot_y * F_left * cos_alpha_left + pos_mot_y * F_right * cos_alpha_right) +
             ( - pos_mot_x * F_left * sin_alpha_left - pos_mot_x * F_right * sin_alpha_right) -
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
    # theta_t = np.arctan2(0-y, 0-x)  # Orientation tangentielle vers le but
    diff_theta = theta-theta_t
    diff_theta = (diff_theta + np.pi) % (2 * np.pi) - np.pi
    d_o = np.linalg.norm(np.array([x, y]) - np.array([0, 0]))
    # return (stage_cost_weight[0] * theta_t +
    #         stage_cost_weight[1] * huber_loss(d_o-10, 5.0) +
    #         stage_cost_weight[2] * (theta -theta_t)**2) 
    return (stage_cost_weight[0] *(x_t-x)**2 +
        stage_cost_weight[1] * (y_t-y)**2 +
        stage_cost_weight[2] * diff_theta**2)


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
        S[k] += 5*terminal_cost(x, target_state, stage_cost_weight)
    return S

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from tf_transformations import euler_from_quaternion


class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.publisherMotorL = self.create_publisher(Float64, '/aquabot/thrusters/left/thrust', 10)
        self.publisherMotorR = self.create_publisher(Float64, '/aquabot/thrusters/right/thrust', 10)
        self.publisherAngleMotorL = self.create_publisher(Float64, '/aquabot/thrusters/left/pos', 10)
        self.publisherAngleMotorR = self.create_publisher(Float64, '/aquabot/thrusters/right/pos', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.mppi=MPPIControllerForAquabot()
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odometry/filtered/map/processed',
            self.odom_callback,
            10)

        self.subscription_path = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10)
        self.subscription_odom
        self.current_state = np.array([30.0, 0.0, 0.0, 0.0,0.0,0.0])
        self.target_state=np.array([-999999,-999999,-999999])
        self.plan=Path()
        self.final_state=np.array([-999999,-999999,-999999])
        self.threshold_target=3.0

    def timer_callback(self):
        if self.target_state[0]!=-999999:
            motorL=Float64()
            motorR=Float64()
            angleMotorL=Float64()
            angleMotorR=Float64()
            # self.get_logger().info('timer: "%f"  "%f"' % (self.current_state[0], self.current_state[1]))
            control, _ = self.mppi.calc_control_input(self.current_state, self.target_state)
            motorL.data=control[0]
            motorR.data=control[1]
            angleMotorL.data=control[2]
            angleMotorR.data=control[3]
            # angleMotorL.data=500.
            # angleMotorR.data=1000.
            # angleMotorL.data=0.
            # angleMotorR.data=0.

            self.get_logger().info('thrust:  "%f"  "%f"' % (control[0], control[1]))
            self.get_logger().info('angle:  "%f"  "%f"' % (angleMotorL.data,  angleMotorR.data))
            self.publisherMotorL.publish(motorL)
            self.publisherMotorR.publish(motorR)
            self.publisherAngleMotorL.publish(angleMotorL)
            self.publisherAngleMotorR.publish(angleMotorR)
        

    def odom_callback(self, msg):

        self.current_state[0]=msg.pose.pose.position.x
        self.current_state[1]=msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.current_state[2]=yaw
        self.current_state[3]=msg.twist.twist.linear.x
        self.current_state[4]=msg.twist.twist.linear.y
        self.current_state[5]=msg.twist.twist.angular.z

        d_target =np.linalg.norm(np.array([self.current_state[0], self.current_state[1]]) - np.array([self.target_state[0], self.target_state[1]]))
        theta_target= self.current_state[2]-self.target_state[2]
        d_final =np.linalg.norm(np.array([self.current_state[0], self.current_state[1]]) - np.array([self.final_state[0], self.final_state[1]]))
        if d_target<self.threshold_target:
            self.new_target()
        if d_final<5.0 :
            self.mppi.sigma=np.array([25, 25, 0.08, 0.08])
            self.mppi.max_thrust=200
            self.threshold_target=3.0
        else : 
            self.mppi.sigma=np.array([60, 60, 0.1, 0.1])
            self.mppi.max_thrust=3000
            self.threshold_target=5.0
        # theta_t = np.arctan2(0-self.current_state[1], 0-self.current_state[0])  # Orientation tangentielle vers le but
        # d_o = np.linalg.norm(np.array([self.current_state[0], self.current_state[1]]) - np.array([0, 0]))
        # self.get_logger().info('d :"%f"  a : "%f" '% (d_o,theta_t,))
        self.get_logger().info('d_final :"%f" d_target :"%f" angle target : "%f"'% (d_final,d_target,theta_target))
        
        # self.get_logger().info('state:"%f"  "%f" "%f"  "%f""%f"  "%f"' % (self.current_state[0], self.current_state[1],self.current_state[2], self.current_state[3],self.current_state[4], self.current_state[5]))
        
        
    def plan_callback(self, msg):
        print("PLAN PLAN PLANPLAN")
        self.plan=msg
        orientation_q = self.plan.poses[-1].pose.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
        x,y=self.plan.poses[-1].pose.position.x,self.plan.poses[-1].pose.position.y
        self.final_state=np.array([x,y,yaw])
        self.new_target()

    def new_target(self) :
        orientation_q = self.plan.poses[0].pose.orientation
        if len(self.plan.poses)>1 :
            self.plan.poses.pop(0)
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
        x,y=self.plan.poses[0].pose.position.x,self.plan.poses[0].pose.position.y
        self.target_state=np.array([x,y,yaw])
        print("NEWTARGET")



def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


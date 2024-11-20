#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

import sys, select, termios, tty

msg_simple = """
Reading from the keyboard and Publishing to Thrusters !
---------------------------
Moving around:
        z     
   q    s    d
        x     

z/x : increase/decrease speed by 10%
q/d : increase/decrease orientation by 5%
s   : reset all to 0

CTRL-C to quit
"""

msg_advanced = """
Reading from the keyboard and Publishing to Thrusters !
---------------------------
Moving around left: | Moving around right:
        z           |         i
   q    s    d      |    j    k    l
        x           |         ,

Left thruster:
z/x : increase/decrease speed by 10%
q/d : increase/decrease orientation by 5%
s   : reset all to 0

Right thruster:
i/, : increase/decrease speed by 10%
j/l : increase/decrease orientation by 5%
k   : reset all to 0

Space : stop both thrusters

CTRL-C to quit
"""

# TeleopKeyboard class take 1 argument: advanced
# Command : ros2 run aquabot_python teleop_keyboard --ros-args -p advanced:=True
class TeleopKeyboard(Node):
    def __init__(self, mode_advanced):
        super().__init__('teleop_keyboard')
        self.left_speed_pub = self.create_publisher(Float64, '/aquabot/thrusters/left/thrust', 5)
        self.right_speed_pub = self.create_publisher(Float64, '/aquabot/thrusters/right/thrust', 5)
        self.left_turn_pub = self.create_publisher(Float64, '/aquabot/thrusters/left/pos', 5)
        self.right_turn_pub = self.create_publisher(Float64, '/aquabot/thrusters/right/pos', 5)
        self.mode_advanced = mode_advanced
        self.get_logger().info('Teleop Keyboard Node Started')
        if mode_advanced:
            print(msg_advanced)
        else:
            print(msg_simple)
        self.settings = termios.tcgetattr(sys.stdin)
        self.run()

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def print_vels(self, mode_advanced, left_speed, left_turn, right_speed, right_turn):
        if mode_advanced:
            print("currently:\tleft_speed %s\tleft_turn %s\t\tright_speed %s\tright_turn %s " % (left_speed, left_turn, right_speed, right_turn))
        else:
            print("currently:\tspeed %s\tturn %s " % (left_speed, left_turn))
    
    def run(self):
        left_speed = 0.0
        left_turn = 0.0
        right_speed = 0.0
        right_turn = 0.0
        speed_limit = 5000.0
        turn_limit = 0.78539816339
        try:
            self.print_vels(self.mode_advanced, left_speed, left_turn, right_speed, right_turn)
            while(1):
                key = self.getKey()
                # Left thruster
                if key == "s":
                    left_speed = 0.0
                    left_turn = 0.0
                    self.print_vels(self.mode_advanced, left_speed, left_turn, right_speed, right_turn)
                elif key == "z":
                    left_speed = min(speed_limit, left_speed + speed_limit * 0.1)
                    self.print_vels(self.mode_advanced, left_speed, left_turn, right_speed, right_turn)
                    if left_speed == speed_limit:
                        print("Speed max limit reached!")
                elif key == "d":
                    left_turn = min(turn_limit, left_turn + turn_limit * 0.05)
                    self.print_vels(self.mode_advanced, left_speed, left_turn, right_speed, right_turn)
                    if left_turn == turn_limit:
                        print("Orientation max limit reached!")
                elif key == "x":
                    left_speed = max(-speed_limit, left_speed - speed_limit * 0.1)
                    self.print_vels(self.mode_advanced, left_speed, left_turn, right_speed, right_turn)
                    if left_speed == speed_limit:
                        print("Speed min limit reached!")
                elif key == "q":
                    left_turn = max(-turn_limit, left_turn - turn_limit * 0.05)
                    self.print_vels(self.mode_advanced, left_speed, left_turn, right_speed, right_turn)
                    if left_turn == turn_limit:
                        print("Orientation min limit reached!")
                # Right thruster
                elif key == "k":
                    right_speed = 0.0
                    right_turn = 0.0
                elif key == "i":
                    right_speed = min(speed_limit, right_speed + speed_limit * 0.1)
                    self.print_vels(self.mode_advanced, left_speed, left_turn, right_speed, right_turn)
                    if right_speed == speed_limit:
                        print("Speed max limit reached!")
                elif key == "l":
                    right_turn = min(turn_limit, right_turn + turn_limit * 0.05)
                    self.print_vels(self.mode_advanced, left_speed, left_turn, right_speed, right_turn)
                    if right_turn == turn_limit:
                        print("Orientation max limit reached!")
                elif key == ";":
                    right_speed = max(-speed_limit, right_speed - speed_limit * 0.1)
                    self.print_vels(self.mode_advanced, left_speed, left_turn, right_speed, right_turn)
                    if right_speed == speed_limit:
                        print("Speed min limit reached!")
                elif key == "j":
                    right_turn = max(-turn_limit, right_turn - turn_limit * 0.05)
                    self.print_vels(self.mode_advanced, left_speed, left_turn, right_speed, right_turn)
                    if right_turn == turn_limit:
                        print("Orientation min limit reached!")
                # Both thrusters stop
                elif key == ' ':
                    left_speed = 0.0
                    left_turn = 0.0
                    right_speed = 0.0
                    right_turn = 0.0
                elif key == '\x03':
                    break

                left_speed_msg = Float64()
                left_turn_msg = Float64()
                left_speed_msg.data = left_speed
                left_turn_msg.data = left_turn

                right_speed_msg = Float64()
                right_turn_msg = Float64()
                right_speed_msg.data = right_speed
                right_turn_msg.data = right_turn
                
                self.left_speed_pub.publish(left_speed_msg)
                self.left_turn_pub.publish(left_turn_msg)
                if self.mode_advanced:
                    self.right_speed_pub.publish(right_speed_msg)
                    self.right_turn_pub.publish(right_turn_msg)
                else:
                    self.right_speed_pub.publish(left_speed_msg)
                    self.right_turn_pub.publish(left_turn_msg)

        except Exception as e:
            print(e)

        finally:
            msg = Float64()
            msg.data = 0.0
            self.print_vels(self.mode_advanced, 0, 0, 0, 0)
            self.left_speed_pub.publish(msg)
            self.right_speed_pub.publish(msg)
            self.left_turn_pub.publish(msg)
            self.right_turn_pub.publish(msg)

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('teleop_keyboard_node')
    node.declare_parameter('advanced', False)
    mode_advanced = node.get_parameter('advanced').get_parameter_value().bool_value
    node.destroy_node()
    teleop_keyboard = TeleopKeyboard(mode_advanced)

    rclpy.spin(teleop_keyboard)
    
    teleop_keyboard.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
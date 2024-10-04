#!/usr/bin/env python3

import rclpy

from aquabot_python.scenario_generator import reopen_scenario

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('reopen_scenario')
    node.declare_parameter('scenario', "")
    node.declare_parameter('obstacles', True)
    scenario = node.get_parameter('scenario').get_parameter_value().string_value
    obstacles = node.get_parameter('obstacles').get_parameter_value().bool_value
    if len(scenario) <= 0:
        scenario = "scenario.sdf"
    reopen_scenario(scenario, obstacles)

if __name__ == "__main__":
    main()
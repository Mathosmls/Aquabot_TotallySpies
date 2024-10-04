#!/usr/bin/env python3

from aquabot_python.scenario_generator import regenerate_scenario_file

def main():
    regenerate_scenario_file("aquabot_windturbines_easy.sdf", False)
    regenerate_scenario_file("aquabot_windturbines_medium.sdf", True)
    regenerate_scenario_file("aquabot_windturbines_hard.sdf", True)

if __name__ == "__main__":
    main()
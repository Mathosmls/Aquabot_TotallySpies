#!/usr/bin/env python3

# This script has to be started from the scripts folder (Aquabot/aquabot_python/scripts)
# It also can be started with the following command: python3 generate_competition_worlds.py

import os
import shutil

from aquabot_python.scenario_generator import generate_scenario

NUM_SCENARIOS = 10
COPY_SCENARIO_NAME = "aquabot_windturbines_medium"

def main():
    base_path = "../../aquabot_gz/worlds/"
    scenario_names = [f'aquabot_windturbines_competition_{i:02d}' for i in range(0, 10)]
    for name in scenario_names:
        print(f"Generating scenario {name}")
        generate_scenario(name)
        filename = name + ".sdf"
        # Copy scenario and replace scenario name in file
        shutil.copyfile(base_path + COPY_SCENARIO_NAME + ".sdf", base_path + filename)
        with open(base_path + filename, 'r') as file:
            filedata = file.read()
            filedata = filedata.replace(COPY_SCENARIO_NAME, name)
            with open(base_path + filename, 'w') as file:
                file.write(filedata)
        # Use existing scenario to generate new one
        with open(base_path + filename, 'r') as file_read:
            lines = file_read.readlines()
            with open(base_path + filename, 'w') as file:
                for line in lines[:-2]:
                    file.write(line)
                    if "<!-- GENERATED SCENARIO -->" in line:
                        break
                with open(filename, 'r') as copy_file:
                    copy_lines = copy_file.readlines()
                    for copy_line in copy_lines[2:]:
                        file.write(copy_line)
        os.remove(filename)
        #os.remove(name + ".png")

if __name__ == "__main__":
    main()

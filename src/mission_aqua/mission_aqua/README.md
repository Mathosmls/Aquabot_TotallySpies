# mission_aqua

## Introduction
**MissionAqua** is a package designed to manage the complete operation for the Aquabot simulation. It includes functionalities such as path planning, autonomous decision-making, QR code recognition, and acoustic sensor-based target detection.


## Features
- **Autonomous Navigation**: Implements a TSP solver to optimize navigation among wind turbines.
- **QR Code Handling**: Identifies and tracks wind turbines via QR codes.
- **Mode Management**: Publishes and manages different operational modes of the USV.
- **Acoustic Sensor Integration**: Locates wind turbines using acoustic range and bearing data.
- **Modular Utilities**: Includes helper functions for mathematical operations and state tracking.


### Initialization
The `MissionAqua` node initializes publishers, subscribers, and data structures required for managing the mission.

#### Key Publishers:
1. **`/goal_target`**: Publishes target positions for USV navigation.
2. **`/goal_pose`**: Communicates the next navigation waypoint.
3. **`/mode`**: Indicates the operational mode of the USV.
4. **`/current_wt`**: Publishes the current wind turbine being processed.

#### Key Subscribers:
1. **`/local_wind_turbine_positions`**: Receives wind turbine positions.
2. **`/pos_gps_align_qr`**: Processes QR code GPS-aligned positions.
3. **`/odometry/filtered/map/processed`**: Tracks the USV's current position.
4. **`/vrx/windturbinesinspection/windturbine_checkup`**: Handles QR code messages.
5. **`/aquabot/sensors/acoustics/receiver/range_bearing`**: Processes acoustic sensor data.


## Utilities

### Core Functions
1. **`tsp_solver`**: Solves the Traveling Salesperson Problem to optimize navigation paths.
2. **`calculate_offset_target`**: Computes target positions offset by a defined radius.
3. **`Quat2yaw`**: Converts quaternions to yaw angles.
4. **`add_or_update_turbine`**: Adds or updates turbine data.
5. **`update_turbine_qr_position`**: Updates turbine QR code positions.
6. **`is_within_distance`**: Checks if the USV is within a specific range of a target.
7. **`find_best_matching_wind_turbine_id`**: Identifies the best turbine based on sensor data.


## Phases of the simulation

### Phase 1: Wind Turbine Exploration
1. Receives turbine positions via `/local_wind_turbine_positions`.
2. Computes optimal navigation order using TSP.
3. Directs the USV to turbines and circles them to scan QR codes.

### Phase 2: Critical Target Identification
1. Awaits range-bearing data to locate a critical wind turbine.
2. Sends the USV to the target position based on QR code and sensor data.

### Phase 3: Final Inspection
1. Performs additional maneuvers to finalize the inspection of critical turbines.
2. Switches operational modes dynamically to handle different scenarios.


## Launch Instructions

To run the node, please execute the following commands (it also works with other worlds):
```
ros2 launch aquabot_gz competition.launch.py world:=aquabot_regatta

ros2 run mission_aqua mission_aqua
```



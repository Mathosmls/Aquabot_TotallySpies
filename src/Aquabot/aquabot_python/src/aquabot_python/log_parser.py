from aquabot_python.replay_container import ReplayContainer, ReplayColumns, ReplayPhase
from aquabot_python.scenario_container import ScenarioContainer, TrajectoryType, TrajectoryPoint, TrajectoryPath, WindturbineContainer, WindturbineState
from aquabot_python.environnement_container import generate_environnement
from aquabot_python.scenario_pyplot import plot_scenario, PlotPoint

import matplotlib.pyplot as plt

def plot_column(replay, column):
    plt.plot(replay.get_column_time(), 
             replay.get_column_from_enum(column), 
             label=replay.get_column_name(column))
    
def plot_graphs(replay, save_path='replay_graphs.png', show=False):
    plt.figure(figsize=(10, 6))

    ################## Phase 1 : SEARCH ##################
    plt.subplot(2, 2, 1)
    try:
        plot_column(replay, ReplayColumns.Windturbine_0_Distance)
        plot_column(replay, ReplayColumns.Windturbine_1_Distance)
        plot_column(replay, ReplayColumns.Windturbine_2_Distance)
        plot_column(replay, ReplayColumns.Windturbine_3_Distance)
        plot_column(replay, ReplayColumns.Windturbine_4_Distance)
    except IndexError:
        pass
    plt.xlabel('Time (s)')
    plt.title('Search Phase - Distance to windturbines')
    try:
        search_phase_index = replay.get_column_from_enum(ReplayColumns.Phase).tolist().index(ReplayPhase.SEARCH.value)
        time_start = replay.get_column_from_enum(ReplayColumns.Time)[search_phase_index]
        plt.axvline(x=time_start, color='green', linestyle='--', label='Start search phase')
    except ValueError:
        print("Phase SEARCH not in replay")
    try:
        rally_phase_index = replay.get_column_from_enum(ReplayColumns.Phase).tolist().index(ReplayPhase.RALLY.value)
        time_end = replay.get_column_from_enum(ReplayColumns.Time)[rally_phase_index]
        plt.axvline(x=time_end, color='red', linestyle='--', label='End search phase')
    except ValueError:
        print("Phase RALLY not in replay")
    plt.legend()

    ################## Phase 2 : RALLY ##################
    plt.subplot(2, 2, 2)
    plot_column(replay, ReplayColumns.Critical_Windturbine_Distance)
    plt.axhline(y=15, color='yellow', linestyle='--', label='Rally tolerance distance') # Plot yellow line at 15 m (tolerance distance)
    plt.title('Rally Phase - Distance to critical windturbine')
    try:
        stabilise_phase_index = replay.get_column_from_enum(ReplayColumns.Phase).tolist().index(ReplayPhase.RALLY.value)
        time_start = replay.get_column_from_enum(ReplayColumns.Time)[stabilise_phase_index]
        plt.axvline(x=time_start, color='green', linestyle='--', label='Start rally phase')
    except ValueError:
        print("Phase RALLY not in replay")
    try:
        stabilise_phase_index = replay.get_column_from_enum(ReplayColumns.Phase).tolist().index(ReplayPhase.STABILISE.value)
        time_end = replay.get_column_from_enum(ReplayColumns.Time)[stabilise_phase_index]
        plt.axvline(x=time_end, color='red', linestyle='--', label='End rally phase')
    except ValueError:
        print("Phase STABILISE not in replay")
    plt.legend()

    ################## Phase 3 : STABILISE ##################
    plt.subplot(2, 2, 3)
    plot_column(replay, ReplayColumns.Target_Pose_Distance)
    plot_column(replay, ReplayColumns.Critical_Windturbine_Heading)
    plt.title('Stabilise Phase - Distance to target and heading')
    try:
        stabilise_phase_index = replay.get_column_from_enum(ReplayColumns.Phase).tolist().index(ReplayPhase.STABILISE.value)
        time_start = replay.get_column_from_enum(ReplayColumns.Time)[stabilise_phase_index]
        plt.axvline(x=time_start, color='green', linestyle='--', label='Start stabilise phase')
    except ValueError:
        print("Phase STABILISE not in replay")
    try:
        turn_around_phase_index = replay.get_column_from_enum(ReplayColumns.Phase).tolist().index(ReplayPhase.TURN_AROUND.value)
        time_end = replay.get_column_from_enum(ReplayColumns.Time)[turn_around_phase_index]
        plt.axvline(x=time_end, color='red', linestyle='--', label='End stabilise phase')
    except ValueError:
        print("Phase TURN AROUND not in replay")
    plt.legend()

    ################## Phase 4 : TURN AROUND ##################
    plt.subplot(2, 2, 4)
    plot_column(replay, ReplayColumns.Critical_Windturbine_Bearing_From_Boat)
    plot_column(replay, ReplayColumns.Critical_Windturbine_Number_Turns)
    plot_column(replay, ReplayColumns.Critical_Windturbine_Distance)
    plt.axhline(y=10, color='red', linestyle='--', label='Target Distance') # Plot red line at 10 m (target distance)
    plt.title('Turn Around Phase - Distance à la turbine critique')
    try:
        turn_around_phase_index = replay.get_column_from_enum(ReplayColumns.Phase).tolist().index(ReplayPhase.TURN_AROUND.value)
        time_start = replay.get_column_from_enum(ReplayColumns.Time)[turn_around_phase_index]
        plt.axvline(x=time_start, color='green', linestyle='--', label='Start turn around phase')
        time_end = replay.get_column_from_enum(ReplayColumns.Time)[-1]
        plt.axvline(x=time_end, color='red', linestyle='--', label='End turn around phase')
    except ValueError:
        print("Phase TURN AROUND not in replay")
    plt.legend()

    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    plt.tight_layout()
    plt.savefig(save_path, dpi=400)
    if show:
        plt.show()

def get_trajectory_path_from_replay(replay, column_x, column_y, column_yaw, type):
    time = replay.get_column_time()
    x = replay.get_column_from_enum(column_x)
    y = replay.get_column_from_enum(column_y)
    if(column_yaw is None):
        yaw = [0] * len(time)
    else:
        yaw = replay.get_column_from_enum(column_yaw)
    path = []
    for i in range(len(time)):
        path.append(TrajectoryPoint([x[i], y[i]], yaw[i], time[i]))
    return TrajectoryPath(type, path)

def get_windturbine_from_replay(replay, column_x, column_y, column_marker_yaw=None):
    x = replay.get_column_from_enum(column_x)
    y = replay.get_column_from_enum(column_y)
    point = [x[0], y[0]]
    state = WindturbineState.OK if column_marker_yaw == None else WindturbineState.CRITICAL
    markers = [0] if column_marker_yaw == None else [replay.get_column_from_enum(column_marker_yaw)[0]]
    windturbine = WindturbineContainer(point, state, 0.0, markers)
    return windturbine

def plot_map(replay, obstacles, save_path="map.png", show=True):
    trajectories = []
    trajectories.append(get_trajectory_path_from_replay(replay, 
        ReplayColumns.Player_X, ReplayColumns.Player_Y, ReplayColumns.Player_Yaw, TrajectoryType.PLAYER))
    windturbines = []
    scenario = ScenarioContainer(windturbines, trajectories)
    point_list = []

    try:
        critical_windturbine_position = [replay.get_column_from_enum(ReplayColumns.Critical_Windturbine_X)[0],
                                        replay.get_column_from_enum(ReplayColumns.Critical_Windturbine_Y)[0]]
        point_list.append(PlotPoint(critical_windturbine_position[0], critical_windturbine_position[1], 'Critical windturbine position', 'o', 'red', 'darkred', 'full', 10))
        critical_target_position = [replay.get_column_from_enum(ReplayColumns.Target_Pose_X)[0],
                                    replay.get_column_from_enum(ReplayColumns.Target_Pose_Y)[0]]
        point_list.append(PlotPoint(critical_target_position[0], critical_target_position[1], 'Target position', 'x', 'red', 'darkred'))
        windturbine_0_position = [replay.get_column_from_enum(ReplayColumns.Windturbine_0_X)[0],
                                  replay.get_column_from_enum(ReplayColumns.Windturbine_0_Y)[0]]
        point_list.append(PlotPoint(windturbine_0_position[0], windturbine_0_position[1], 'Windturbine 0 position', 'o', 'orange', 'darkorange'))
        windturbine_1_position = [replay.get_column_from_enum(ReplayColumns.Windturbine_1_X)[0],
                                    replay.get_column_from_enum(ReplayColumns.Windturbine_1_Y)[0]]
        point_list.append(PlotPoint(windturbine_1_position[0], windturbine_1_position[1], 'Windturbine 1 position', 'o', 'orange', 'darkorange'))
        windturbine_2_position = [replay.get_column_from_enum(ReplayColumns.Windturbine_2_X)[0],
                                    replay.get_column_from_enum(ReplayColumns.Windturbine_2_Y)[0]]
        point_list.append(PlotPoint(windturbine_2_position[0], windturbine_2_position[1], 'Windturbine 2 position', 'o', 'orange', 'darkorange'))
        windturbine_3_position = [replay.get_column_from_enum(ReplayColumns.Windturbine_3_X)[0],
                                    replay.get_column_from_enum(ReplayColumns.Windturbine_3_Y)[0]]
        point_list.append(PlotPoint(windturbine_3_position[0], windturbine_3_position[1], 'Windturbine 3 position', 'o', 'orange', 'darkorange'))
        windturbine_4_position = [replay.get_column_from_enum(ReplayColumns.Windturbine_4_X)[0],
                                    replay.get_column_from_enum(ReplayColumns.Windturbine_4_Y)[0]]
        point_list.append(PlotPoint(windturbine_4_position[0], windturbine_4_position[1], 'Windturbine 4 position', 'o', 'orange', 'darkorange'))
    except IndexError:
        pass
    try:
        rally_phase_index = replay.get_column_from_enum(ReplayColumns.Phase).tolist().index(ReplayPhase.RALLY.value)
        last_search_player_position = [replay.get_column_from_enum(ReplayColumns.Player_X)[rally_phase_index],
                                      replay.get_column_from_enum(ReplayColumns.Player_Y)[rally_phase_index]]
        point_list.append(PlotPoint(last_search_player_position[0], last_search_player_position[1], 'Player position at rally', '^', 'blue', 'darkblue'))
        stabilise_phase_index = replay.get_column_from_enum(ReplayColumns.Phase).tolist().index(ReplayPhase.STABILISE.value)
        last_rally_player_position = [replay.get_column_from_enum(ReplayColumns.Player_X)[stabilise_phase_index],
                                      replay.get_column_from_enum(ReplayColumns.Player_Y)[stabilise_phase_index]]
        point_list.append(PlotPoint(last_rally_player_position[0], last_rally_player_position[1], 'Player position at stabilise', '^', 'green', 'darkgreen'))
        turnaround_phase_index = replay.get_column_from_enum(ReplayColumns.Phase).tolist().index(ReplayPhase.TURN_AROUND.value)
        last_rally_player_position = [replay.get_column_from_enum(ReplayColumns.Player_X)[turnaround_phase_index],
                                      replay.get_column_from_enum(ReplayColumns.Player_Y)[turnaround_phase_index]]
        point_list.append(PlotPoint(last_rally_player_position[0], last_rally_player_position[1], 'Player position at turn around', '^', 'yellow', 'yellow'))
    except ValueError:
        print("Phase not found in replay")
    # Plot scenario
    plot_scenario(scenario, generate_environnement(obstacles), save_path=save_path, show=show, point_list=point_list)

def plot_log_csv(filepath, obstacles=True):
    replay = ReplayContainer(filepath)
    plot_graphs(replay, "replay_graphs.png", show=True)
    plot_map(replay, obstacles, "replay_map.png", show=True)
    
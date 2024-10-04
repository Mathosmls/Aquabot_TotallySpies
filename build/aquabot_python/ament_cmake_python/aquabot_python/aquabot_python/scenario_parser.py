# This is xml parser for scenario file
import xml.etree.ElementTree as ET
from aquabot_python.scenario_container import ScenarioContainer, TrajectoryPath, TrajectoryType, TrajectoryPoint, WindturbineContainer, WindturbineState
from aquabot_python.generator_methods import get_distance_between_points, get_windturbine_markers
import math

ACTOR_ENEMY = 'ennemy'
ACTOR_ALLY = 'ally_'
WINDTURBINE_MAIN_NAME = "aquabot_windturbine_main"
WINDTURBINE_ROTOR_NAME = "aquabot_windturbine_rotor"
WINDTURBINE_MARKER_NAME = "aquabot_marker"

def deserialize(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()
    world = root.find('world')
    windturbines = parse_windturbines(world)
    return ScenarioContainer(windturbines)

def parse_trajectory_paths(root):
    # Actor tag contains trajectory path
    trajectory_paths = []
    for actor in root.findall('actor'):
        name = actor.get('name')
        if name == ACTOR_ENEMY:
            trajectory_paths.append(parse_trajectory_path(actor, TrajectoryType.ENEMY))
        elif name.startswith(ACTOR_ALLY):
            trajectory_paths.append(parse_trajectory_path(actor, TrajectoryType.ALLY))
    return trajectory_paths

def parse_trajectory_path(actor, type):
    # Trajectory path is stored in script/trajectory tag
    trajectory_path = TrajectoryPath(type, [])
    trajectory = actor.find('script').find('trajectory')
    for waypoint in trajectory.findall('waypoint'):
        time = float(waypoint.find('time').text)
        pose = waypoint.find('pose').text
        pose = pose.split()
        trajectory_path.path.append(TrajectoryPoint([float(pose[0]), float(pose[1])], float(pose[5]), time))
    return trajectory_path

def parse_pinger_position(root):
    for plugin in root.findall('plugin'):
        if plugin.get('name') == 'vrx::WindturbinesInspectionScoringPlugin':
            position = plugin.find('pinger_position').text.split()
            return [float(position[0]), float(position[1])]

def parse_windturbine_markers(root, windturbine_id):
    markers = []
    for include in sorted(root.findall('include'), key=lambda x: x.find('name').text):
        name = include.find('name').text
        if name.startswith(WINDTURBINE_MARKER_NAME + "_" + str(windturbine_id)):
            pose = include.find('pose').text.split()
            markers.append(float(pose[5]))
    return markers

def parse_windturbines(root):
    critical_pos = parse_pinger_position(root)
    windturbines = []
    for include in sorted(root.findall('include'), key=lambda x: x.find('name').text):
        name = include.find('name').text
        if name.startswith(WINDTURBINE_MAIN_NAME):
            id = name[len(WINDTURBINE_MAIN_NAME)+1:].split('_')[0]
            split_pose = include.find('pose').text.split()
            point = [float(split_pose[0]), float(split_pose[1])]
            yaw = float(split_pose[5])
            if get_distance_between_points(point, critical_pos) < 0.1:
                state = WindturbineState.CRITICAL
            elif name.endswith("OK"):
                state = WindturbineState.OK
            else:
                state = WindturbineState.KO
            marker_yaws = parse_windturbine_markers(root, id)
            windturbines.append(WindturbineContainer(point, state, yaw, marker_yaws))
    return windturbines

def serialize(scenario_container, file_path, seed=None):
    critical_marker_name = ""
    # Then check to fix the orientation of the actors
    sdf = ET.Element('sdf')
    root = ET.SubElement(sdf, 'world')
    # Seed
    if seed is not None:
        ET.SubElement(root, 'generation_seed').text = str(seed)
    # Platform
    platform = ET.SubElement(root, 'include')
    ET.SubElement(platform, 'name').text = 'platform'
    ET.SubElement(platform, 'uri').text = 'platform'
    # Windturbines
    id_windturbine = 0
    for windturbine in scenario_container.windturbines:
        suffix = "_" + str(id_windturbine) + "_"
        if windturbine.state == WindturbineState.OK:
            suffix += "OK"
        else:
            suffix += "KO"
        # Main (include)
        include = ET.SubElement(root, 'include')
        ET.SubElement(include, 'name').text = WINDTURBINE_MAIN_NAME + suffix
        ET.SubElement(include, 'pose').text = '%f %f 0 0 0 %f' % (windturbine.point[0], windturbine.point[1], windturbine.yaw)
        ET.SubElement(include, 'uri').text = WINDTURBINE_MAIN_NAME
        # Rotor (actor)
        actor = ET.SubElement(root, 'actor')
        actor.set('name', WINDTURBINE_ROTOR_NAME + suffix)
        skin = ET.SubElement(actor, 'skin')
        ET.SubElement(skin, 'filename').text = '../models/aquabot_windturbine_rotor/mesh/rotor.dae'
        script = ET.SubElement(actor, 'script')
        ET.SubElement(script, 'loop').text = 'true'
        ET.SubElement(script, 'delay_start').text = '2'
        ET.SubElement(script, 'auto_start').text = 'true'
        trajectory = ET.SubElement(script, 'trajectory')
        trajectory.set('id', '0')
        trajectory.set('type', 'square')
        rotations = [0, -3.1415, -6.283]
        time_per_rotation = 15
        for i in range(len(rotations)):
            waypoint = ET.SubElement(trajectory, 'waypoint')
            ET.SubElement(waypoint, 'time').text = '%f' % (i*time_per_rotation)
            ET.SubElement(waypoint, 'pose').text = '%f %f 30.0 %.4f 0 %f' % (windturbine.point[0], windturbine.point[1], rotations[i], windturbine.yaw)
        # Markers
        id_marker = 0
        for marker_yaw in windturbine.marker_yaws:
            marker = ET.SubElement(root, 'include')
            marker_name = WINDTURBINE_MARKER_NAME + suffix + "_" + str(id_marker)
            ET.SubElement(marker, 'name').text = marker_name
            ET.SubElement(marker, 'pose').text = '%f %f 0 0 0 %f' % (windturbine.point[0], windturbine.point[1], marker_yaw)
            ET.SubElement(marker, 'uri').text = WINDTURBINE_MARKER_NAME + suffix
            if(windturbine.state == WindturbineState.CRITICAL):
                critical_marker_name = marker_name
            id_marker += 1
        id_windturbine += 1

    # Scoring
    scoring = ET.SubElement(root, 'plugin')
    scoring.set('filename', 'libWindturbinesInspectionScoringPlugin.so')
    scoring.set('name', 'vrx::WindturbinesInspectionScoringPlugin')
    ET.SubElement(scoring, 'vehicle').text = 'aquabot'
    ET.SubElement(scoring, 'task_name').text = 'windturbines_inspection'
    ET.SubElement(scoring, 'task_info_topic').text = '/vrx/task/info'
    ET.SubElement(scoring, 'initial_state_duration').text = '10'
    ET.SubElement(scoring, 'ready_state_duration').text = '10'
    ET.SubElement(scoring, 'running_state_duration').text = '1200'
    ET.SubElement(scoring, 'release_topic').text = '/vrx/release'
    for windturbine in scenario_container.windturbines:
        if windturbine.state == WindturbineState.CRITICAL:
            ET.SubElement(scoring, 'pinger_position').text = '%f %f 0' % (windturbine.point[0], windturbine.point[1])
    ET.SubElement(scoring, 'phase_search_max_time').text = '600'
    ET.SubElement(scoring, 'phase_rally_period_publish').text = '20.0'
    ET.SubElement(scoring, 'phase_rally_min_distance').text = '15.0'
    ET.SubElement(scoring, 'phase_turnaround_number_turns').text = '-1.0'
    ET.SubElement(scoring, 'phase_stabilize_total_time').text = '180'
    ET.SubElement(scoring, 'phase_inspection_target_distance').text = '10.0'
    ET.SubElement(scoring, 'phase_stabilize_turnaround_mean_period').text = '30'
    ET.SubElement(scoring, 'collision_min_distance').text = '10.0'
    ET.SubElement(scoring, 'collision_finish').text = 'true'
    ET.SubElement(scoring, 'collision_penality').text = '10'
    ET.SubElement(scoring, 'critical_marker_name').text = critical_marker_name
    windturbines = ET.SubElement(scoring, 'windturbines')
    id_windturbine = 0
    for windturbine in scenario_container.windturbines:
        windturbine_element = ET.SubElement(windturbines, 'windturbine')
        ET.SubElement(windturbine_element, 'id').text = str(id_windturbine)
        status = "OK" if windturbine.state == WindturbineState.OK else "KO"
        ET.SubElement(windturbine_element, 'status').text = status
        ET.SubElement(windturbine_element, 'name').text = WINDTURBINE_MAIN_NAME + "_" + str(id_windturbine) + "_" + status
        id_windturbine += 1
    tree = ET.ElementTree(sdf)
    ET.indent(tree, space="  ")
    tree.write(file_path, encoding='utf-8', xml_declaration=False)

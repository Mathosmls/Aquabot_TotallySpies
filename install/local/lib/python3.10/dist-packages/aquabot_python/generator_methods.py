import random
import numpy

from aquabot_python.scenario_container import WindturbineState, WindturbineContainer, ScenarioContainer
from aquabot_python.environnement_container import EnvironnementContainer
import math

# Constants
RETRY_MAX = 1000
MIN_DISTANCE_BETWEEN_WINDTURBINES = 100
MIN_WINDTURBINE_DISTANCE_FROM_PLAYER = 80
MAX_WINDTURBINE_DISTANCE_FROM_PLAYER = 250

# Enum : PointFindType
from enum import Enum

class PointFindType(Enum):
    INSIDE_MAP_NGZ = 0
    DISTANCE_MIN_MAX = 1
    DISTANCE_ANGLE_MIN_MAX = 2

# Simple methods
def get_angle_between_points(point1, point2):
    return math.atan2(point2[1]-point1[1], point2[0]-point1[0])

def get_distance_between_points(point1, point2):
    return math.sqrt((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2)

def is_point_in_map(map_cords, point):
    return point[0] >= map_cords[0] and point[0] <= map_cords[1] and point[1] >= map_cords[2] and point[1] <= map_cords[3]

def is_segement_intersecting_circle(circle_position, circle_radius, segment_start, segment_end):
    # Segment AB and circle center C
    AB = numpy.subtract(segment_end, segment_start)
    AC = numpy.subtract(circle_position, segment_start)
    # Projection of AC on AB
    j = numpy.dot(AC, AB) / numpy.dot(AB, AB)
    D = (segment_start[0]+j*AB[0], segment_start[1]+j*AB[1])
    AD = numpy.subtract(D, segment_start)
    # Solve k : AD = k*AB
    k = numpy.dot(AD, AB) / numpy.dot(AB, AB)
    # If k is between 0 and 1, the projection is on the segment
    if k >= 0 and k <= 1:
        # Distance between the projection and the circle center
        distance = numpy.linalg.norm(numpy.subtract(D, circle_position))
        return distance < circle_radius
    else:
        if k < 0:
            distance = numpy.linalg.norm(numpy.subtract(segment_start, circle_position))
        else:
            distance = numpy.linalg.norm(numpy.subtract(segment_end, circle_position))
        return distance < circle_radius

def is_inside_circle(circle_position, circle_radius, position):
    return (position[0]-circle_position[0])**2 + (position[1]-circle_position[1])**2 < circle_radius**2

def is_point_in_no_go_zones(no_go_zones, point):
    for no_go_zone in no_go_zones:
        if is_inside_circle(no_go_zone.center, no_go_zone.radius, point):
            return True
    return False

# Random point in map
# Conditions : point must not be in a no go zone
#              point must be in map
# Conditions not always necessary : point must be at a minimum/maximum distance from another point
#                                   point must be in a random angle from a previous point
def get_random_point_in_map(environnement_container, point_find_type=PointFindType.INSIDE_MAP_NGZ, origin_point=(0,0), yaw_point=0, distance_min_max=(0,0), angle_min_max=(0,2*math.pi)):
    count = 0
    angle = 0
    while True:
        count += 1
        if point_find_type == PointFindType.INSIDE_MAP_NGZ:
            point = (random.uniform(environnement_container.map_borders[0], environnement_container.map_borders[1]),
                        random.uniform(environnement_container.map_borders[2], environnement_container.map_borders[3]))
        elif point_find_type == PointFindType.DISTANCE_MIN_MAX or point_find_type == PointFindType.DISTANCE_ANGLE_MIN_MAX:
            angle = yaw_point + (random.uniform(angle_min_max[0], angle_min_max[1]) * ((random.randint(0,1)*2)-1))
            distance = random.uniform(distance_min_max[0], distance_min_max[1])
            point = (origin_point[0] + distance*math.cos(angle),
                        origin_point[1] + distance*math.sin(angle))
        else:
            print("Error: Invalid PointFindType")
            return None, None
        
        if is_point_in_map(environnement_container.map_borders, point) and not is_point_in_no_go_zones(environnement_container.no_go_zones, point):
            if point_find_type == PointFindType.DISTANCE_MIN_MAX or point_find_type == PointFindType.DISTANCE_ANGLE_MIN_MAX:
                for i in range(len(environnement_container.no_go_zones)):
                    if is_segement_intersecting_circle(environnement_container.no_go_zones[i].center, environnement_container.no_go_zones[i].radius, origin_point, point):
                        break
                else:
                    return point, angle
            else:
                return point, angle
        
        if count > RETRY_MAX:
            print("Error: get_random_point_in_map(): count > %d" % RETRY_MAX)
            return None, None

# Random windturbine states
def get_random_windturbines_states(num_turbines):
    # 1 critical turbine needed
    id_critical = random.randint(0, num_turbines-1)
    list_states = []
    for i in range(num_turbines):
        if i == id_critical:
            list_states.append(WindturbineState.CRITICAL)
        elif random.random() > 0.5:
            list_states.append(WindturbineState.OK)
        else:
            list_states.append(WindturbineState.KO)
    return list_states

def get_windturbine_markers():
    yaw_markers = []
    num_markers_per_windturbine = 1
    yaw_marker = random.uniform(0, 2*math.pi)
    i = 0
    for i in range(num_markers_per_windturbine):
        yaw_markers.append(((2*math.pi/num_markers_per_windturbine)*i + yaw_marker) % (2*math.pi))
        i+=1
    return yaw_markers

# Random windturbine
def get_random_windturbine(state, environnement_container, scenario_container):
    too_close = True
    count = 0
    point = (0,0)
    while too_close:
        count+=1
        point, angle = get_random_point_in_map(environnement_container, point_find_type=PointFindType.DISTANCE_MIN_MAX, distance_min_max=(MIN_WINDTURBINE_DISTANCE_FROM_PLAYER, MAX_WINDTURBINE_DISTANCE_FROM_PLAYER))
        too_close = False
        for other in scenario_container.windturbines:
            if get_distance_between_points(point, other.point) < MIN_DISTANCE_BETWEEN_WINDTURBINES:
                too_close = True
        if count > RETRY_MAX:
            return None
    yaw = random.uniform(0, 2*math.pi)
    yaw_markers = get_windturbine_markers()
    return WindturbineContainer(point, state, yaw, yaw_markers)

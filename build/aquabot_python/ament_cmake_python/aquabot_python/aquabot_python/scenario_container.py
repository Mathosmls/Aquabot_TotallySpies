# This class contains all informations about scenario for aquabot competition
from enum import Enum

class TrajectoryType(Enum):
    ENEMY = 1
    ALLY = 2
    PLAYER = 3
    ALERT = 4

class TrajectoryPoint:
    def __init__(self, point, yaw, time):
        self.point = point
        self.yaw = yaw
        self.time = time

class TrajectoryPath:
    def __init__(self, type, path):
        self.path = path
        self.type = type

class WindturbineState(Enum):
    CRITICAL = 0
    OK = 1
    KO = 2

class WindturbineContainer:
    def __init__(self, point, state, yaw, marker_yaws):
        self.point = point
        self.state = state
        self.yaw = yaw
        self.marker_yaws = marker_yaws

class ScenarioContainer:
    def __init__(self, windturbines, trajectory_paths = []):
        self.trajectory_paths = trajectory_paths
        self.windturbines = windturbines

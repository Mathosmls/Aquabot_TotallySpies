# This class contains all informations about replay log file for aquabot competition
from enum import Enum, auto
import numpy as np

# Enum pour définir les colonnes
class ReplayColumns(Enum):
    Time = 0
    Phase = auto()
    Player_X = auto()
    Player_Y = auto()
    Player_Yaw = auto()
    Player_Speed = auto()
    Player_Distance = auto()
    Target_Pose_X = auto()
    Target_Pose_Y = auto()
    Target_Pose_Yaw = auto()
    Target_Pose_Distance = auto()
    Critical_Windturbine_X = auto()
    Critical_Windturbine_Y = auto()
    Critical_Windturbine_Distance = auto()
    Critical_Windturbine_Heading = auto()
    Critical_Windturbine_Bearing_From_Boat = auto()
    Critical_Windturbine_Number_Turns = auto()
    Windturbine_0_X = auto()
    Windturbine_0_Y = auto()
    Windturbine_0_Distance = auto()
    Windturbine_0_Checkup_Status = auto()
    Windturbine_1_X = auto()
    Windturbine_1_Y = auto()
    Windturbine_1_Distance = auto()
    Windturbine_1_Checkup_Status = auto()
    Windturbine_2_X = auto()
    Windturbine_2_Y = auto()
    Windturbine_2_Distance = auto()
    Windturbine_2_Checkup_Status = auto()
    Windturbine_3_X = auto()
    Windturbine_3_Y = auto()
    Windturbine_3_Distance = auto()
    Windturbine_3_Checkup_Status = auto()
    Windturbine_4_X = auto()
    Windturbine_4_Y = auto()
    Windturbine_4_Distance = auto()
    Windturbine_4_Checkup_Status = auto()
    
class ReplayPhase(Enum):
    INITIAL = 0
    SEARCH = 1
    RALLY = 2
    STABILISE = 3
    TURN_AROUND = 4
    FINISHED = 5

class ReplayContainer:
    def __init__(self, csv_file, txt_file=None):
        # Load csv file to numpy array
        self.data = np.genfromtxt(csv_file, delimiter=',', skip_header=1)

    def get_column_from_enum(self, column):
        return self.data[:, column.value]
    
    def get_column_time(self):
        return self.get_column_from_enum(ReplayColumns.Time)
    
    def get_column_name(self, column):
        return column.name.replace('_', ' ')



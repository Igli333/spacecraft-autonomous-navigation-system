from enum import Enum


class Phase(Enum):
    FAR = 0
    MID = 1
    CLOSE = 2
    DOCKING = 3
    DOCKED = 4
    ABORT = 5
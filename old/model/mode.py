from enum import Enum


class Mode(Enum):
    FAR_APPROACH = 0            # Efficient range reduction
    MID_APPROACH = 1            # Align corridor and reduce lateral error
    CLOSE_APPROACH = 2          # Slow approach
    HOLD = 3                    # Maintain relative position
    DOCK = 4                    # Align for docking
    ABORT = 5                   # Retreat

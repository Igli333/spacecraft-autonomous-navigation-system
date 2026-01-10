import numpy as np

from model import actuation_command, analysis, mode


class Planner:
    def __init__(self):
        self.mode = "FAR_APPROACH"

        self.command = actuation_command.ActuationCommand()

        self.close_range = 20.0
        self.docking_range = 1.0

    def plan(self, a: analysis.Analysis) -> actuation_command.ActuationCommand:
        r = a.range

        if a.constraints_violated:
            self.mode = mode.Mode.ABORT
        elif r > self.close_range:
            self.mode =  mode.Mode.FAR_APPROACH
        elif r > self.docking_range:
            self.mode = mode.Mode.CLOSE_APPROACH
        else:
            self.mode = mode.Mode.DOCK

        # Default for now, no logic applied
        force = np.zeros(3)
        torque = np.zeros(3)

        return actuation_command.ActuationCommand(force, torque, self.mode)

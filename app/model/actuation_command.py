from .mode import Mode


class ActuationCommand:
    def __init__(self, desired_acceleration, torque, mode=Mode.FAR_APPROACH):
        self.desired_acceleration = desired_acceleration
        self.torque_B = torque
        self.mode = mode

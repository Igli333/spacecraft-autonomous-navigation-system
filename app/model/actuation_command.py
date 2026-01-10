from mode import Mode


class ActuationCommand:
    def __init__(self, force, torque, mode=Mode.FAR_APPROACH):
        self.force_B = force
        self.torque_B = torque
        self.mode = mode

    def setForce(self, force):
        self.force_B = force

    def setTorque(self, torque):
        self.torque_B = torque

    def setMode(self, mode: Mode):
        self.mode = mode

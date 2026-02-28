from Basilisk.simulation.extForceTorque import ExtForceTorque


class Executor:
    def __init__(self, forceEffector: ExtForceTorque):
        self.ModelTag = "executor"
        self.forceEffector = forceEffector

    def execute(
            self,
            force,
            torque
    ):
        self.forceEffector.extForce_B = force
        self.forceEffector.extTorquePntB_B = torque

        # print(f"{self.forceEffector.extForce_B} -- {self.forceEffector.extTorquePntB_B}")

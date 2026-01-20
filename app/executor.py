from Basilisk.simulation.extForceTorque import ExtForceTorque


# TODO: Improve
class Executor:
    def __init__(self, forceEffector: ExtForceTorque):
        self.ModelTag = "executor"
        self.forceEffector = forceEffector

    def execute(
            self,
            force,
            torque,
            CurrentSimNans
    ):
        self.forceEffector.extForce_B = force.tolist()
        self.forceEffector.extTorque_B = torque.tolist()

        print(f"{self.forceEffector.extForce_B} -- {self.forceEffector.extTorque_B}")

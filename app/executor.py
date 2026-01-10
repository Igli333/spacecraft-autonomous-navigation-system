from model import actuation_command
from Basilisk.simulation.extForceTorque import ExtForceTorque

# TODO: Improve
class Executor:
    def __init__(self, forceEffector: ExtForceTorque):
        self.ModelTag = "executor"
        self.forceEffector = forceEffector

        self.maxForce = 0.05  # N

    def execute(self, actuationCommand: actuation_command.ActuationCommand, CurrentSimNans):
        self.applyForce(actuationCommand.force_B)
        self.applyTorque(actuationCommand.torque_B)

        print(f'Force executed: {actuationCommand.force_B}')
        print(f'Torque executed: {actuationCommand.torque_B}')

    def applyForce(self, force_B):
        self.forceEffector.extForce_B = force_B.tolist()

    def applyTorque(self, torque_B):
        self.forceEffector.torque_B = torque_B.tolist()

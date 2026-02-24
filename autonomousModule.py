import os
import requests
import numpy as np

from dotenv import load_dotenv

from Basilisk.architecture import sysModel
from Basilisk.simulation.extForceTorque import ExtForceTorque
from Basilisk.simulation.simpleNav import SimpleNav

load_dotenv('sim.env')


class MAPEK_Module(sysModel.SysModel):
    def __init__(self,
                 target: SimpleNav,
                 chaser: SimpleNav,
                 forceEffector: ExtForceTorque,
                 modelTag="autonomous"):
        super().__init__()
        self.ModelTag = modelTag

        self.forceEffector = forceEffector

        self.targetNavInMsg = target.transOutMsg.read()
        self.chaserNavInMsg = chaser.transOutMsg.read()
        self.targetAttMsg = target.attOutMsg.read()
        self.chaserAttMsg = chaser.attOutMsg.read()
        self.chaser_mass = float(os.getenv('CHASER_MASS'))

    def UpdateState(self, CurrentSimNanos):
        time = int(CurrentSimNanos * 1e-9)

        request_body = {
            'rT': np.array(self.targetNavInMsg.r_BN_N).tolist(),
            'vT': np.array(self.targetNavInMsg.v_BN_N).tolist(),
            'rC': np.array(self.chaserNavInMsg.r_BN_N).tolist(),
            'vC': np.array(self.chaserNavInMsg.v_BN_N).tolist(),
            'sigma_BN_C': np.array(self.chaserAttMsg.sigma_BN).tolist(),
            'sigma_BN_T': np.array(self.targetAttMsg.sigma_BN).tolist(),
            'omega_BN_C': np.array(self.chaserAttMsg.omega_BN_B).tolist(),
            'omega_BN_T': np.array(self.targetAttMsg.omega_BN_B).tolist(),
            'chaser_mass': self.chaser_mass,
            'time': time,
        }

        commandData = requests.post(f'{os.getenv("MONITOR_URI")}/monitor', json=request_body, timeout=10).json()
        force, torque = commandData['force'], commandData['torque']

        # Find a way to stop the simulation
        # self.simulation.StopSimulation()

        self.forceEffector.extForce_B = force
        self.forceEffector.extTorquePntB_B = torque

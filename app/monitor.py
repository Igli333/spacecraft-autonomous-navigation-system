import numpy as np

from .model import monitor_latest
from Basilisk.utilities import orbitalMotion, RigidBodyKinematics as rbk


class Monitor:
    def __init__(self, targetMsg, chaserMsg, targetAttMsg, chaserAttMsg):
        super().__init__()
        self.ModelTag = "monitor"
        self.targetNavInMsg = targetMsg
        self.chaserNavInMsg = chaserMsg
        self.targetAttMsg = targetAttMsg
        self.chaserAttMsg = chaserAttMsg

        self.latest = None

    def updateState(self):
        target = self.targetNavInMsg()
        chaser = self.chaserNavInMsg()
        targetAtt = self.targetAttMsg()
        chaserAtt = self.chaserAttMsg()

        rT = np.array(target.r_BN_N)
        vT = np.array(target.v_BN_N)
        rC = np.array(chaser.r_BN_N)
        vC = np.array(chaser.v_BN_N)

        # Relative inertial
        relR = rC - rT
        relV = vC - vT

        # Hill frame
        HN = orbitalMotion.hillFrame(rT, vT)
        rho = HN @ relR
        rhoDot = HN @ relV

        sigma_BN_C = np.array(chaserAtt.sigma_BN)
        sigma_BN_T = np.array(targetAtt.sigma_BN)

        omega_BN_C = np.array(chaserAtt.omega_BN_B)
        omega_BN_T = np.array(targetAtt.omega_BN_B)

        BN_C = rbk.MRP2C(sigma_BN_C)
        BN_T = rbk.MRP2C(sigma_BN_T)

        BT = BN_C @ BN_T.T
        sigma_BT = rbk.C2MRP(BT)
        omega_BT = omega_BN_C - BT @ omega_BN_T

        # Store
        return monitor_latest.MonitorLatestData(
            rho=rho,
            rhoDot=rhoDot,
            range_=np.linalg.norm(rho),
            r_target=rT,
            v_target=vT,
            sigma_BT=sigma_BT,
            omega_BT=omega_BT,
            BN_T=BN_T,
        )

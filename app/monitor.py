import numpy as np

from Basilisk.utilities import orbitalMotion


class Monitor:
    def __init__(self):
        super().__init__()
        self.ModelTag = "monitor"
        self.targetNavInMsg = None
        self.chaserNavInMsg = None

        self.relPos_H = []
        self.relVel_H = []
        self.range = []

        self.r_target = []
        self.v_target = []
        self.r_chaser = []
        self.v_chaser = []

    def subscribeTo(self, targetMsg, chaserMsg):
        self.targetNavInMsg = targetMsg
        self.chaserNavInMsg = chaserMsg

    def updateState(self, CurrentSimNanos):
        target = self.targetNavInMsg()
        chaser = self.chaserNavInMsg()

        rT = np.array(target.r_BN_N)
        vT = np.array(target.v_BN_N)
        rC = np.array(chaser.r_BN_N)
        vC = np.array(chaser.v_BN_N)

        self.r_target.append(rT)
        self.v_target.append(vT)
        self.r_chaser.append(rC)
        self.v_chaser.append(vC)

        # Relative inertial
        relR = rC - rT
        relV = vC - vT

        # Hill frame
        HN = orbitalMotion.hillFrame(rT, vT)
        rho_H = HN @ relR
        rhoDot_H = HN @ relV

        # Metrics
        rng = np.linalg.norm(rho_H)

        # Store
        self.relPos_H.append(rho_H)
        self.relVel_H.append(rhoDot_H)
        self.range.append(rng)

    def getLatest(self):
        return {
            "r_target": self.r_target[-1],
            "v_target": self.v_target[-1],
            "r_chaser": self.r_chaser[-1],
            "v_chaser": self.v_chaser[-1],
            "relPos_H": self.relPos_H[-1],
            "relVel_H": self.relVel_H[-1],
            "range": self.range[-1]
        }

import numpy as np

from .phase import Phase


class Analysis:
    def __init__(
            self,
            rho: np.ndarray,
            rhoDot: np.ndarray,
            range_: float,
            safe: bool,
            phase_: Phase,
            meanMotion,
            sigma_BT,
            omega_BT,
            dock_ready,
    ):
        self.rho = rho
        self.rhoDot = rhoDot
        self.range = range_
        self.safe = safe
        self.phase = phase_
        self.meanMotion = meanMotion
        self.sigma_BT = sigma_BT
        self.omega_BT = omega_BT
        self.dock_ready = dock_ready

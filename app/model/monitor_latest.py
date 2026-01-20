class MonitorLatestData:
    def __init__(
            self,
            rho,
            rhoDot,
            range_,
            r_target,
            sigma_BT,
            omega_BT
    ):
        self.rho = rho
        self.rhoDot = rhoDot
        self.range = range_
        self.r_target = r_target
        self.sigma_BT = sigma_BT
        self.omega_BT = omega_BT

class MonitorLatestData:
    def __init__(
            self,
            rho,
            rhoDot,
            range_,
            r_target,
            v_target,
            sigma_BT,
            omega_BT,
            BN_T
    ):
        self.rho = rho
        self.rhoDot = rhoDot
        self.range = range_
        self.r_target = r_target
        self.sigma_BT = sigma_BT
        self.omega_BT = omega_BT
        self.v_target = v_target
        self.BN_T = BN_T

import numpy as np

from .model import analysis, phase


class Planner:
    def __init__(self, kb, chaser_mass):
        self.kb = kb
        self.mass = chaser_mass
        self.docked = False

    def plan(self, a: analysis.Analysis, time):
        force = self.planForce(a, time)
        torque = self.planTorque(a, time)

        return force, torque

    def planForce(self, a: analysis.Analysis, time):
        if not a.safe:
            K_safe = 5e-5  # N/m
            F_H = -K_safe * a.rho
            return self._saturate(F_H)

        gains = {
            phase.Phase.FAR: (1e-4, 5e-3),
            phase.Phase.MID: (3e-4, 1e-2),
            phase.Phase.CLOSE: (8e-4, 3e-2),
            phase.Phase.DOCKING: (2e-4, 5e-2)
        }

        Kp, Kd = gains[a.phase]
        v_max = {
            phase.Phase.FAR: 0.5,  # m/s
            phase.Phase.MID: 0.2,
            phase.Phase.CLOSE: 0.05,
            phase.Phase.DOCKING: 0.01
        }[a.phase]

        rho = a.rho
        rhoDot = a.rhoDot
        dist = np.linalg.norm(rho) + 1e-6

        v_des = -v_max * rho / dist
        vel_error = rhoDot - v_des

        n = a.meanMotion
        a_ff = np.array([
            3 * n ** 2 * rho[0] + 2 * n * rhoDot[1],
            -2 * n * rhoDot[0],
            -n ** 2 * rho[2]
        ])

        a_cmd = a_ff - Kp * rho - Kd * vel_error
        F_H = self.mass * a_cmd

        # Saturate
        F_max = {
            phase.Phase.FAR: 20.0,
            phase.Phase.MID: 10.0,
            phase.Phase.CLOSE: 2.0,
            phase.Phase.DOCKING: 0.5
        }[a.phase]

        F_mag = np.linalg.norm(F_H)
        if F_mag > F_max:
            F_H = F_H / F_mag * F_max

        return F_H

        # self.kb.log(run_id=0, time=time,
        #             rho_x=a.rho[0], rho_y=a.rho[1], rho_z=a.rho[2],
        #             rhoDot_x=a.rhoDot[0], rhoDot_y=a.rhoDot[1], rhoDot_z=a.rhoDot[2],
        #             force_x=force[0], force_y=force[1], force_z=force[2],
        #             phase=a.phase, risk=a.safe,
        #             success=None, abort=0, abort_reason=None)

        return self._saturate(F_H)

    def planTorque(self, a: analysis.Analysis, time):
        ph = a.phase

        if ph not in [phase.Phase.CLOSE, phase.Phase.DOCKING]:
            return np.zeros(3)

        sigma_BT = a.sigma_BT
        omega_BT = a.omega_BT

        Kr = 5.0
        Kw = 50.0

        tau = -Kr * sigma_BT - Kw * omega_BT

        tau_max = 0.5
        mag = np.linalg.norm(tau)
        if mag > tau_max:
            tau = tau / mag * tau_max

        return tau

    def _saturate(self, F_H):
        """
        Saturate force in Hill frame
        """
        F_max = 5.0  # N per axis (realistic)
        return np.clip(F_H, -F_max, F_max)

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
        # Saturate
        F_max = {
            phase.Phase.FAR: 20.0,
            phase.Phase.MID: 10.0,
            phase.Phase.CLOSE: 2.0,
            phase.Phase.DOCKING: 0.5
        }[a.phase]

        if not a.safe:
            K_safe = 5e-5  # N/m
            F_H = -K_safe * a.rho
            F_H = self._saturate(F_H, F_max)
        elif a.abort or a.success:
            F_H = np.zeros(3)
        elif a.phase in [phase.Phase.CLOSE, phase.Phase.DOCKING] and not a.capture_possible:
            n_hat = np.array([1.0, 0.0, 0.0])

            # Parallel components
            dist = np.dot(a.rho, n_hat)
            v_parallel = np.dot(a.rhoDot, n_hat)

            v_perp = a.rhoDot - v_parallel * n_hat

            # Lyapunov gains
            k_r = 8e-5  # position shaping
            k_v = 3e-4  # velocity damping
            k_lat = 5e-4  # lateral damping

            a_cap = (
                    -k_r * dist * n_hat
                    - k_v * v_parallel * n_hat
                    - k_lat * v_perp
            )

            F_H = self.mass * a_cap

            F_max_learned = self.kb.learnedMaxForce(np.linalg.norm(a.rho))
            if F_max_learned:
                F_H = self._saturate(F_H, min(F_max, F_max_learned))

            F_H = self._saturate(F_H, F_max)
        else:
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

            F_max_learned = self.kb.learnedMaxForce(np.linalg.norm(a.rho))
            if F_max_learned:
                F_max = min(F_max, F_max_learned)

            F_H = self._saturate(F_H, F_max)

        if a.phase in [phase.Phase.MID, phase.Phase.CLOSE, phase.Phase.DOCKING]:
            self.kb.log_force(
                time=time,  # ns to s
                rho_x=a.rho[0], rho_y=a.rho[1], rho_z=a.rho[2],
                rhoDot_x=a.rhoDot[0], rhoDot_y=a.rhoDot[1], rhoDot_z=a.rhoDot[2],
                force_x=F_H[0], force_y=F_H[1], force_z=F_H[2],
                phase=str(a.phase.name), risk=a.safe,
                success=a.success, abort=a.abort, abort_reason=a.abort_reason
            )

        return F_H

    def planTorque(self, a: analysis.Analysis, time):
        dist = np.linalg.norm(a.rho)

        sigma_norm = np.linalg.norm(a.sigma_BT)
        omega_norm = np.linalg.norm(a.omega_BT)

        p_abort = self.kb.abortProbabilityTorque(dist, sigma_norm, omega_norm)

        if p_abort > 0.1 or a.phase not in [phase.Phase.CLOSE, phase.Phase.DOCKING]:
            tau = np.zeros(3)
        else:
            Kr = 5.0
            Kw = 50.0

            tau = -Kr * a.sigma_BT - Kw * a.omega_BT

            tau_max = 0.5
            tau_learned = self.kb.learnedMaxTorque(dist)
            if tau_learned:
                tau_max = min(tau_max, tau_learned)

            mag = np.linalg.norm(tau)
            if mag > tau_max:
                tau = tau / mag * tau_max

        if a.phase in [phase.Phase.MID, phase.Phase.CLOSE, phase.Phase.DOCKING]:
            self.kb.log_torque(
                time=time, dist=dist,
                sigma_x=a.sigma_BT[0], sigma_y=a.sigma_BT[1], sigma_z=a.sigma_BT[2],
                omega_x=a.omega_BT[0], omega_y=a.omega_BT[1], omega_z=a.omega_BT[2],
                torque_x=tau[0], torque_y=tau[1], torque_z=tau[2],
                phase=str(a.phase.name), success=a.success,
                abort=a.abort, abort_reason=a.abort_reason
            )

        return tau

    def _saturate(self, F_H, F_max):
        F_mag = np.linalg.norm(F_H)
        if F_mag > F_max:
            return F_H / F_mag * F_max

        return F_H

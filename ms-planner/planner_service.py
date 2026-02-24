import os

import numpy as np
import requests

FAR = 'FAR'
MID = 'MID'
CLOSE = 'CLOSE'
DOCKING = 'DOCKING'


def plan(a):
    rho = np.array(a['rho'])
    dist = np.linalg.norm(rho)
    phase = a['phase']
    force = planForce(a, rho, phase, dist)
    torque = planTorque(a, dist, phase)

    return {
        'force': force,
        'torque': torque
    }


def planForce(a, rho, phase, dist):
    rhoDot = np.array(a['rhoDot'])
    chaser_mass = a['chaser_mass']

    F_max = {
        FAR: float(os.getenv('F_MAX_FAR', 20.0)),
        MID: float(os.getenv('F_MAX_MID', 10.0)),
        CLOSE: float(os.getenv('F_MAX_CLOSE', 2.0)),
        DOCKING: float(os.getenv('F_MAX_DOCKING', 0.5))
    }[phase]

    if not a['safe']:
        K_safe = float(os.getenv('K_SAFE', 5e-5))  # N/m
        F_H = -K_safe * rho
        F_H = _saturate(F_H, F_max)
    elif a['abort'] or a['success']:
        F_H = np.zeros(3)
    elif phase in [CLOSE, DOCKING] and not a['capture_possible']:
        n_hat = np.array([
            float(os.getenv('DOCKING_AXIS_X')),
            float(os.getenv('DOCKING_AXIS_Y')),
            float(os.getenv('DOCKING_AXIS_Z'))
        ])

        # Parallel components
        dist = np.dot(rho, n_hat)
        v_parallel = np.dot(rhoDot, n_hat)

        v_perp = rhoDot - v_parallel * n_hat

        # Lyapunov gains
        k_r = float(os.getenv('K_R', 8e-5))  # position shaping
        k_v = float(os.getenv('K_V', 3e-4))  # velocity damping
        k_lat = float(os.getenv('K_LAT', 5e-4))  # lateral damping

        a_cap = (
                -k_r * dist * n_hat
                - k_v * v_parallel * n_hat
                - k_lat * v_perp
        )

        F_H = chaser_mass * a_cap

        F_max_learned = requests.get(
            f"{os.getenv("KNOWLEDGE_BASE_URI")}/analytics/max-force?range={np.linalg.norm(rho)}"
        ).json()

        if F_max_learned["value"]:
            F_H = _saturate(F_H, min(F_max, F_max_learned["value"]))

        F_H = _saturate(F_H, F_max)
    else:
        gains = {
            FAR: (float(os.getenv('KP_FAR', 1e-4)), float(os.getenv('KD_FAR', 5e-3))),
            MID: (float(os.getenv('KP_MID', 3e-4)), float(os.getenv('KD_MID', 1e-2))),
            CLOSE: (float(os.getenv('KP_CLOSE', 8e-4)), float(os.getenv('KD_CLOSE', 3e-2))),
            DOCKING: (float(os.getenv('KP_DOCKING', 2e-4)), float(os.getenv('KD_DOCKING', 5e-2)))
        }

        Kp, Kd = gains[phase]
        v_max = {
            FAR: float(os.getenv('V_MAX_FAR', 0.5)),  # m/s
            MID: float(os.getenv('V_MAX_MID', 0.2)),
            CLOSE: float(os.getenv('V_MAX_CLOSE', 0.05)),
            DOCKING: float(os.getenv('V_MAX_DOCKING', 0.01))
        }[phase]

        dist = dist + 1e-6

        v_des = -v_max * rho / dist
        vel_error = rhoDot - v_des

        n = a['meanMotion']
        a_ff = np.array([
            3 * n ** 2 * rho[0] + 2 * n * rhoDot[1],
            -2 * n * rhoDot[0],
            -n ** 2 * rho[2]
        ])

        a_cmd = a_ff - Kp * rho - Kd * vel_error
        F_H = chaser_mass * a_cmd

        F_max_learned = requests.get(
            f"{os.getenv("KNOWLEDGE_BASE_URI")}/analytics/max-force?range={np.linalg.norm(rho)}"
        ).json()

        if F_max_learned["value"]:
            F_max = min(F_max, F_max_learned["value"])

        F_H = _saturate(F_H, F_max)

    if phase in [MID, CLOSE, DOCKING]:
        requests.post(
            f"{os.getenv("KNOWLEDGE_BASE_URI")}/log/force",
            json={
                "time": a['time'],

                "rho_x": rho[0],
                "rho_y": rho[1],
                "rho_z": rho[2],

                "rhoDot_x": rhoDot[0],
                "rhoDot_y": rhoDot[1],
                "rhoDot_z": rhoDot[2],

                "force_x": F_H[0],
                "force_y": F_H[1],
                "force_z": F_H[2],

                "phase": phase,
                "risk": a['safe'],

                "success": a['success'],
                "abort": a['abort'],
                "abort_reason": a['abort_reason']
            }
        )

    return F_H.tolist()


def planTorque(a, dist, phase):
    sigma_BT = np.array(a['sigma_BT'])
    omega_BT = np.array(a['omega_BT'])

    sigma_norm = np.linalg.norm(sigma_BT)
    omega_norm = np.linalg.norm(omega_BT)

    p_abort = requests.get(
        f"{os.getenv("KNOWLEDGE_BASE_URI")}/analytics/abort-probability-torque?dist={dist}&sigma_norm={sigma_norm}&omega_norm={omega_norm}"
    ).json()

    if p_abort['value'] > float(os.getenv('P_ABORT_THRESHOLD', 0.1)) or phase not in [CLOSE, DOCKING]:
        tau = np.zeros(3)
    else:
        Kr = float(os.getenv('K_R_TORQUE', 5.0))
        Kw = float(os.getenv('K_W_TORQUE', 50.0))

        tau = -Kr * sigma_BT - Kw * omega_BT

        tau_max = float(os.getenv('TAU_MAX', 0.5))
        tau_learned = requests.get(
            f"{os.getenv("KNOWLEDGE_BASE_URI")}/analytics/max-torque?dist={dist}"
        ).json()

        if tau_learned["value"]:
            tau_max = min(tau_max, tau_learned["value"])

        mag = np.linalg.norm(tau)
        if mag > tau_max:
            tau = tau / mag * tau_max

    if phase in [CLOSE, DOCKING]:
        requests.post(
            f"{os.getenv("KNOWLEDGE_BASE_URI")}/log/torque",
            json={
                "time": a['time'],
                "dist": dist,

                "sigma_x": sigma_BT[0],
                "sigma_y": sigma_BT[1],
                "sigma_z": sigma_BT[2],

                "omega_x": omega_BT[0],
                "omega_y": omega_BT[1],
                "omega_z": omega_BT[2],

                "torque_x": tau[0],
                "torque_y": tau[1],
                "torque_z": tau[2],

                "phase": phase,
                "success": a['success'],
                "abort": a['abort'],
                "abort_reason": a['abort_reason']
            }
        )
    return tau.tolist()


def _saturate(F_H, F_max):
    F_mag = np.linalg.norm(F_H)
    if F_mag > F_max:
        return F_H / F_mag * F_max

    return F_H

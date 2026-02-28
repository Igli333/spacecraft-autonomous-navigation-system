import os

import numpy as np
import requests

FAR = 'FAR'
MID = 'MID'
CLOSE = 'CLOSE'
DOCKING = 'DOCKING'

kb_uri = os.getenv('KNOWLEDGE_BASE_URI')


def plan(a, config):
    rho = np.array(a['rho'])
    dist = np.linalg.norm(rho)
    phase = a['phase']

    force = planForce(a, rho, phase, dist, config['force_control'])
    torque = planTorque(a, dist, phase, config['torque_control'])

    return {
        'force': force,
        'torque': torque
    }


def planForce(a, rho, phase, dist, config):
    rhoDot = np.array(a['rhoDot'])
    chaser_mass = a['chaser_mass']

    F_max = config["max_force_n"][phase]

    # Unsafe
    if not a['safe']:
        K_safe = config["k_safe"]
        F_H = -K_safe * rho
        F_H = _saturate(F_H, F_max)

    # Abort or Success
    elif a['abort'] or a['success']:
        F_H = np.zeros(3)

    # Capture shaping
    elif phase in [CLOSE, DOCKING] and not a['capture_possible']:

        n_hat = np.array(config["docking_axis"])

        dist_parallel = np.dot(rho, n_hat)
        v_parallel = np.dot(rhoDot, n_hat)
        v_perp = rhoDot - v_parallel * n_hat

        k_r = config["lyapunov_gains"]["k_r"]
        k_v = config["lyapunov_gains"]["k_v"]
        k_lat = config["lyapunov_gains"]["k_lat"]

        a_cap = (
                -k_r * dist_parallel * n_hat
                - k_v * v_parallel * n_hat
                - k_lat * v_perp
        )

        F_H = chaser_mass * a_cap

        learned = requests.get(
            f"{kb_uri}/analytics/max-force?range={dist}"
        ).json()

        if learned.get("value"):
            F_max = min(F_max, learned["value"])

        F_H = _saturate(F_H, F_max)

    # Nominal PD control
    else:
        gains = config["pd_gains"][phase]
        Kp = gains["kp"]
        Kd = gains["kd"]

        v_max = config["velocity_limits_m_s"][phase]

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

        learned = requests.get(
            f"{kb_uri}/analytics/max-force?range={dist}"
        ).json()

        if learned.get("value"):
            F_max = min(F_max, learned["value"])

        F_H = _saturate(F_H, F_max)

    return F_H.tolist()


def planTorque(a, dist, phase, torque_config):
    sigma_BT = np.array(a['sigma_BT'])
    omega_BT = np.array(a['omega_BT'])

    sigma_norm = np.linalg.norm(sigma_BT)
    omega_norm = np.linalg.norm(omega_BT)

    p_abort = requests.get(
        f"{kb_uri}/analytics/abort-probability-torque?dist={dist}&sigma_norm={sigma_norm}&omega_norm={omega_norm}"
    ).json()

    abort_threshold = torque_config["abort_threshold"]

    if p_abort.get("value", 0) > abort_threshold or phase not in [CLOSE, DOCKING]:
        tau = np.zeros(3)

    else:
        Kr = torque_config["k_r"]
        Kw = torque_config["k_w"]
        tau_max = torque_config["tau_max"]

        tau = -Kr * sigma_BT - Kw * omega_BT

        learned = requests.get(
            f"{kb_uri}/analytics/max-torque?dist={dist}"
        ).json()

        if learned.get("value"):
            tau_max = min(tau_max, learned["value"])

        mag = np.linalg.norm(tau)
        if mag > tau_max:
            tau = tau / mag * tau_max

    return tau.tolist()


def _saturate(F_H, F_max):
    F_mag = np.linalg.norm(F_H)
    if F_mag > F_max:
        return F_H / F_mag * F_max
    return F_H

import math
import os

import requests
import numpy as np

D2R = math.pi / 180.0

FAR = 'FAR'
MID = 'MID'
CLOSE = 'CLOSE'
DOCKING = 'DOCKING'


def analyze(data, config):
    mu = config["mu"]
    alpha = config["alpha"]
    epsilon_risk = config["epsilon_risk"]
    min_range = config["min_range"]
    n_hat = np.array(config["docking_axis"])
    kb_uri = os.getenv("KNOWLEDGE_BASE_URI")

    PHASE_FAR_THRESHOLD = config["phase_thresholds_m"][FAR]
    PHASE_MID_THRESHOLD = config["phase_thresholds_m"][MID]
    PHASE_CLOSE_THRESHOLD = config["phase_thresholds_m"][CLOSE]

    DOCK_POS_THRESHOLD = config["docking_tolerances"]["position_m"]
    DOCK_VEL_THRESHOLD = config["docking_tolerances"]["velocity_m_s"]
    DOCK_ATT_THRESHOLD = config["docking_tolerances"]["attitude_deg"] * D2R
    DOCK_RATE_THRESHOLD = config["docking_tolerances"]["rate_deg_s"] * D2R

    rho = np.array(data['rho'])
    rhoDot = np.array(data['rhoDot'])
    range_ = data['range']
    sigma_BT = np.array(data['sigma_BT'])
    omega_BT = np.array(data['omega_BT'])

    dist = np.dot(rho, n_hat)
    speed = np.linalg.norm(rhoDot)
    v_parallel = np.dot(rhoDot, n_hat)

    capture_possible = requests.get(
        f'{kb_uri}/analytics/success-envelope',
        params={"dist": dist, "window": v_parallel}
    ).json()

    if range_ > PHASE_FAR_THRESHOLD:
        phase = FAR
    elif range_ > PHASE_MID_THRESHOLD:
        phase = MID
    elif range_ > PHASE_CLOSE_THRESHOLD:
        phase = CLOSE
    else:
        phase = DOCKING

    physics_v_max = alpha * np.sqrt(max(range_, min_range))

    kb_v_max = requests.get(
        f'{kb_uri}/analytics/safe-speed',
        params={"range": range_}
    ).json()

    kb_value = kb_v_max.get("value")
    v_safe = min(physics_v_max, kb_value) if kb_value else physics_v_max

    risk = speed / v_safe
    safe = risk < epsilon_risk

    # Orbital mean motion
    r = np.linalg.norm(np.array(data['r_target']))
    n = np.sqrt(mu / r ** 3)

    pos_ready = dist < DOCK_POS_THRESHOLD
    vel_ready = speed < DOCK_VEL_THRESHOLD
    att_ready = np.linalg.norm(sigma_BT) < DOCK_ATT_THRESHOLD
    rate_ready = np.linalg.norm(omega_BT) < DOCK_RATE_THRESHOLD

    dock_ready = pos_ready and vel_ready and att_ready and rate_ready

    success = False
    abort = False
    abort_reason = None

    if phase != FAR:
        if dock_ready:
            success = True
        elif not safe and range_ < 5.0:
            abort = True
            abort_reason = "unsafe_close_range"

    return {
        'range': range_,
        'rho': rho.tolist(),
        'rhoDot': rhoDot.tolist(),
        'safe': bool(safe),
        'phase': phase,
        'meanMotion': n,
        'sigma_BT': sigma_BT.tolist(),
        'omega_BT': omega_BT.tolist(),
        'dock_ready': bool(dock_ready),
        'abort': bool(abort),
        'abort_reason': abort_reason,
        'success': bool(success),
        'capture_possible': bool(capture_possible.get('value')),
        'chaser_mass': data['chaser_mass'],
        'time': data['time'],
    }

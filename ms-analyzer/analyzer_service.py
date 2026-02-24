import os
import math
import requests
import numpy as np


mu = float(os.getenv('MU', 3.986004418e14))
D2R = (math.pi / 180.)

FAR = 'FAR'
MID = 'MID'
CLOSE = 'CLOSE'
DOCKING = 'DOCKING'

# Phase thresholds
PHASE_FAR = float(os.getenv('PHASE_FAR', 50))
PHASE_MID = float(os.getenv('PHASE_MID', 10))
PHASE_CLOSE = float(os.getenv('PHASE_CLOSE', 1))

# Docking tolerances
DOCK_POS_THRESHOLD = float(os.getenv('DOCK_POS_THRESHOLD', 0.1))
DOCK_VEL_THRESHOLD = float(os.getenv('DOCK_VEL_THRESHOLD', 0.01))
DOCK_ATT_THRESHOLD = float(os.getenv('DOCK_ATT_THRESHOLD_DEG', 5)) * D2R
DOCK_RATE_THRESHOLD = float(os.getenv('DOCK_RATE_THRESHOLD_DEG', 0.5)) * D2R


def analyze(data):
    rho = np.array(data['rho'])
    rhoDot = np.array(data['rhoDot'])
    range_ = data['range']
    sigma_BT = np.array(data['sigma_BT'])
    omega_BT = np.array(data['omega_BT'])

    # Target docking axis
    n_hat = np.array([
        float(os.getenv('DOCKING_AXIS_X')),
        float(os.getenv('DOCKING_AXIS_Y')),
        float(os.getenv('DOCKING_AXIS_Z'))
    ])

    dist = np.dot(rho, n_hat)
    speed = np.linalg.norm(rhoDot)

    v_parallel = np.dot(rhoDot, n_hat)
    capture_possible = requests.get(
        f'{os.getenv("KNOWLEDGE_BASE_URI")}/analytics/success-envelope?dist={dist}&window={v_parallel}'
    ).json()

    if range_ > PHASE_FAR:
        phase = FAR
    elif range_ > PHASE_MID:
        phase = MID
    elif range_ > PHASE_CLOSE:
        phase = CLOSE
    else:
        phase = DOCKING

    physics_v_max = float(os.getenv('ALPHA', 0.1)) * np.sqrt(max(range_, float(os.getenv('MIN_RANGE', 0.1))))
    kb_v_max = requests.get(
        f'{os.getenv("KNOWLEDGE_BASE_URI")}/analytics/safe-speed?range={range_}'
    ).json()

    v_safe = min(physics_v_max, kb_v_max['value']) if kb_v_max['value'] else physics_v_max

    risk = speed / v_safe

    r = np.linalg.norm(np.array(data['r_target']))
    n = np.sqrt(mu / r ** 3)

    safe = risk < float(os.getenv('EPSILON_RISK'))

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
        'capture_possible': bool(capture_possible['value']),
        'chaser_mass': data['chaser_mass'],
        'time': data['time'],
    }

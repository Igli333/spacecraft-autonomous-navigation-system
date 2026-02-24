import utils
import numpy as np


def monitor(data):
    # Relative inertial
    rT = np.array(data['rT'])
    vT = np.array(data['vT'])
    relR = np.array(data['rC'] - rT)
    relV = np.array(data['vC'] - vT)

    HN = utils.hillFrame(rT, vT)

    rho = HN @ relR
    rhoDot = HN @ relV

    BN_C = utils.MRP2C(data['sigma_BN_C'])
    BN_T = utils.MRP2C(data['sigma_BN_T'])

    BT = BN_C @ BN_T.T
    sigma_BT = utils.C2MRP(BT)
    omega_BT = np.array(data['omega_BN_C']) - BT @ np.array(data['omega_BN_T'])

    payload = {
        'rho': rho.tolist(),
        'rhoDot': rhoDot.tolist(),
        'range': float(np.linalg.norm(rho)),
        'r_target': rT.tolist(),
        'v_target': vT.tolist(),
        'sigma_BT': sigma_BT.tolist(),
        'omega_BT': omega_BT.tolist(),
        'BN_T': BN_T.tolist(),
        'chaser_mass': data['chaser_mass'],
        'time': data['time']
    }

    return payload

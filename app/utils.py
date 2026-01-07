import numpy as np
import matplotlib.pyplot as plt

from Basilisk.utilities import orbitalMotion, macros
from Basilisk.simulation.spacecraft import Spacecraft

from matplotlib.animation import FuncAnimation


def applyInitialRandomization(target: Spacecraft, chaser: Spacecraft, docking_port_N, seed=None):
    if seed is not None:
        np.random.seed(seed)

    # ---- Hill-frame relative position (meters)
    rho_H = np.array([
        np.random.uniform(-50.0, 50.0),  # radial
        np.random.uniform(-150.0, 150.0),  # along-track
        np.random.uniform(-30.0, 30.0)  # cross-track
    ])

    # ---- Hill-frame relative velocity (m/s)
    rhoDot_H = np.array([
        np.random.uniform(-0.02, 0.02),
        np.random.uniform(-0.02, 0.02),
        np.random.uniform(-0.01, 0.01)
    ])

    rT = np.array(target.hub.r_CN_NInit, dtype=float).reshape(3, )
    vT = np.array(target.hub.v_CN_NInit, dtype=float).reshape(3, )

    HN = orbitalMotion.hillFrame(rT, vT)

    chaser.hub.r_CN_NInit = list(rT + HN.T @ rho_H)
    chaser.hub.v_CN_NInit = list(vT + HN.T @ rhoDot_H)

    # ---- Random chaser attitude (±5 deg)
    att_sigma = 5.0 * macros.D2R
    sigma = np.random.uniform(-att_sigma, att_sigma, size=3)
    chaser.hub.sigma_BNInit = [[sigma[0]], [sigma[1]], [sigma[2]]]

    # ---- Random chaser body rates (±0.1 deg/s)
    rate_sigma = 0.1 * macros.D2R
    omega = np.random.uniform(-rate_sigma, rate_sigma, size=3)
    chaser.hub.omega_BN_BInit = [[omega[0]], [omega[1]], [omega[2]]]


def animateRelativeMotion(rel_positions, docking_port_H=None):
    """
    rel_positions: list of 3-element arrays in Hill frame (chaser w.r.t target)
    docking_port_H: optional 3-element array, position of docking port in Hill frame
    """
    rel_positions = np.array(rel_positions)
    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'b-', label='Chaser trajectory')
    point, = ax.plot([], [], 'ro', label='Docking port')

    ax.set_xlim(-150, 150)
    ax.set_ylim(-20, 20)
    ax.set_xlabel("Along-track (m)")
    ax.set_ylabel("Radial (m)")
    ax.set_title("Chaser Relative Motion (Hill frame)")
    ax.grid()
    ax.legend()

    def update(frame):
        line.set_data(rel_positions[:frame, 1], rel_positions[:frame, 0])
        if docking_port_H is not None:
            point.set_data([docking_port_H[1]], [docking_port_H[0]])
        return line, point

    ani = FuncAnimation(fig, update, frames=len(rel_positions), interval=50, blit=True)
    plt.show()

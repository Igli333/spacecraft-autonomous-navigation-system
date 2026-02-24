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
        np.random.uniform(-25.0, 25.0),  # radial
        np.random.uniform(-100.0, 100.0),  # along-track
        np.random.uniform(-15.0, 15.0)  # cross-track
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


def animateChaserTarget(target_pos, chaser_pos, zoom_margin=200):
    """
    Animate chaser and target positions in absolute and relative frames.

    target_pos: (N,3) array of target positions
    chaser_pos: (N,3) array of chaser positions
    zoom_margin: meters to add around the zoomed absolute plot
    """
    target_pos = np.array(target_pos).reshape(len(target_pos), 3)
    chaser_pos = np.array(chaser_pos).reshape(len(chaser_pos), 3)
    N = len(target_pos)
    assert chaser_pos.shape[0] == N, "Target and chaser must have same length"

    # Relative positions (chaser w.r.t target)
    rel_pos = chaser_pos - target_pos

    fig, axes = plt.subplots(1, 2, figsize=(14, 7))

    # --- Absolute motion plot (zoomed) ---
    ax_abs = axes[0]
    line_target_abs, = ax_abs.plot([], [], 'g--', lw=1, label='Target')
    point_target_abs, = ax_abs.plot([], [], 'go')
    line_chaser_abs, = ax_abs.plot([], [], 'b-', lw=2, label='Chaser')
    point_chaser_abs, = ax_abs.plot([], [], 'bo')
    vector_abs, = ax_abs.plot([], [], 'r:', lw=1, label='Relative vector')

    # Zoom axes based on initial positions ± margin
    x_all = np.concatenate([target_pos[:, 0], chaser_pos[:, 0]])
    y_all = np.concatenate([target_pos[:, 1], chaser_pos[:, 1]])
    ax_abs.set_xlim(x_all.min() - zoom_margin, x_all.max() + zoom_margin)
    ax_abs.set_ylim(y_all.min() - zoom_margin, y_all.max() + zoom_margin)
    ax_abs.set_aspect('equal')
    ax_abs.set_xlabel("X (m)")
    ax_abs.set_ylabel("Y (m)")
    ax_abs.set_title("Absolute Motion (Zoomed)")
    ax_abs.grid()
    ax_abs.legend()
    ax_abs.axhline(0, color='k', lw=0.5)
    ax_abs.axvline(0, color='k', lw=0.5)

    # --- Relative motion plot (Hill-frame style) ---
    ax_rel = axes[1]
    line_rel, = ax_rel.plot([], [], 'b-', lw=2, label='Chaser trajectory')
    point_rel, = ax_rel.plot([], [], 'bo', label='Chaser current')
    point_target_rel, = ax_rel.plot(0, 0, 'ro', label='Target')
    vector_rel, = ax_rel.plot([], [], 'r:', lw=1, label='Relative vector')

    max_range = np.max(np.abs(rel_pos[:, :2])) * 1.2
    lim = max_range if max_range > 1e-3 else 10
    ax_rel.set_xlim(-lim, lim)
    ax_rel.set_ylim(-lim, lim)
    ax_rel.set_aspect('equal')
    ax_rel.set_xlabel("Along-track (m)")
    ax_rel.set_ylabel("Radial (m)")
    ax_rel.set_title("Relative Motion (Target-centered)")
    ax_rel.grid()
    ax_rel.legend()
    ax_rel.axhline(0, color='k', lw=0.5)
    ax_rel.axvline(0, color='k', lw=0.5)

    def update(frame):
        # Absolute plot
        line_target_abs.set_data(target_pos[:frame, 0], target_pos[:frame, 1])
        point_target_abs.set_data([target_pos[frame - 1, 0]], [target_pos[frame - 1, 1]])
        line_chaser_abs.set_data(chaser_pos[:frame, 0], chaser_pos[:frame, 1])
        point_chaser_abs.set_data([chaser_pos[frame - 1, 0]], [chaser_pos[frame - 1, 1]])
        vector_abs.set_data(
            [target_pos[frame - 1, 0], chaser_pos[frame - 1, 0]],
            [target_pos[frame - 1, 1], chaser_pos[frame - 1, 1]]
        )

        # Relative plot
        line_rel.set_data(rel_pos[:frame, 1], rel_pos[:frame, 0])
        point_rel.set_data([rel_pos[frame - 1, 1]], [rel_pos[frame - 1, 0]])
        vector_rel.set_data([0, rel_pos[frame - 1, 1]], [0, rel_pos[frame - 1, 0]])

        return (line_target_abs, point_target_abs, line_chaser_abs, point_chaser_abs, vector_abs,
                line_rel, point_rel, point_target_rel, vector_rel)

    ani = FuncAnimation(fig, update, frames=N, interval=10, blit=False)
    plt.show()
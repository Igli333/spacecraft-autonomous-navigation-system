import numpy as np

from Basilisk.utilities import macros
from .model import analysis, phase, monitor_latest
from .knowledge_base import KnowledgeBase


class Analyzer:
    def __init__(self, kb: KnowledgeBase):
        self.kb = kb

    def analyze(self, data: monitor_latest.MonitorLatestData):
        rho = data.rho
        rhoDot = data.rhoDot
        range_ = data.range
        sigma_BT = data.sigma_BT
        omega_BT = data.omega_BT

        n_hat = np.array([1.0, 0.0, 0.0])  # target docking axis
        dist = np.dot(rho, n_hat)

        speed = np.linalg.norm(rhoDot)

        v_parallel = np.dot(rhoDot, n_hat)
        capture_possible = self.kb.captureSuccessEnvelope(dist, v_parallel)

        if range_ > 50:
            phase_ = phase.Phase.FAR
        elif range_ > 10:
            phase_ = phase.Phase.MID
        elif range_ > 1:
            phase_ = phase.Phase.CLOSE
        else:
            phase_ = phase.Phase.DOCKING

        physics_v_max = 0.1 * np.sqrt(max(range_, 0.1))
        kb_v_max = self.kb.learnedSafeSpeed(range_)

        v_safe = min(physics_v_max, kb_v_max) if kb_v_max else physics_v_max

        risk = speed / v_safe

        mu = 3.986004418e14  # Earth GM
        r = np.linalg.norm(data.r_target)
        n = np.sqrt(mu / r ** 3)

        safe = risk < 0.1

        pos_ready = dist < 0.1
        vel_ready = speed < 0.01
        att_ready = np.linalg.norm(sigma_BT) < 5 * macros.D2R
        rate_ready = np.linalg.norm(omega_BT) < 0.5 * macros.D2R

        dock_ready = pos_ready and vel_ready and att_ready and rate_ready

        success = False
        abort = False
        abort_reason = None

        if phase_ != phase.Phase.FAR:
            if dock_ready:
                success = True
            elif not safe and range_ < 5.0:
                abort = True
                abort_reason = "unsafe_close_range"

        return analysis.Analysis(
            range_=range_,
            rho=rho,
            rhoDot=rhoDot,
            safe=safe,
            phase_=phase_,
            meanMotion=n,
            sigma_BT=sigma_BT,
            omega_BT=omega_BT,
            dock_ready=dock_ready,
            abort=abort,
            abort_reason=abort_reason,
            success=success,
            capture_possible=capture_possible
        )

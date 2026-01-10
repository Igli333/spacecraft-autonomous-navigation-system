import numpy as np

from monitor import Monitor
from model import analysis


class Analyzer:
    def __init__(self):
        self.analysis = {}
        self.monitor = None

        self.max_closing_rate = 0.1  # m/s
        self.max_lateral_offset = 2.0  # m
        self.max_range = 200.0  # m

    def analyze(self, monitor: Monitor):
        data = monitor.getLatest()

        rho = data["relPos_H"]
        rhoDot = data["relVel_H"]

        range_ = data["range"]
        closing_rate = rhoDot[0]  # radial in Hill frame
        lateral_offset = np.linalg.norm(rho[1:])

        inside_corridor = lateral_offset < self.max_lateral_offset
        closing_rate_ok = abs(closing_rate) < self.max_closing_rate
        range_ok = range_ < self.max_range

        constraints_violated = not (
                inside_corridor and closing_rate_ok and range_ok
        )

        return analysis.Analysis(
            range_=range_,
            closing_rate=closing_rate,
            lateral_offset=lateral_offset,
            inside_corridor=inside_corridor,
            constraints_violated=constraints_violated,
            relPos_H=rho,
            relVel_H=rhoDot,
        )

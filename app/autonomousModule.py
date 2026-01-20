from .monitor import Monitor
from .analyzer import Analyzer
from .planner import Planner
from .executor import Executor
from .knowledge_base import KnowledgeBase
from .model import actuation_command, analysis

from Basilisk.architecture import sysModel
from Basilisk.simulation.extForceTorque import ExtForceTorque
from Basilisk.simulation.simpleNav import SimpleNav


class MAPEK_Module(sysModel.SysModel):
    def __init__(self,
                 target: SimpleNav,
                 chaser: SimpleNav,
                 forceEffector: ExtForceTorque,
                 chaser_mass: float,
                 modelTag="autonomous"):
        super().__init__()
        self.ModelTag = modelTag
        self.forceEffector = forceEffector
        self.knowledge = KnowledgeBase()

        self.monitor = Monitor(
            target.transOutMsg.read,
            chaser.transOutMsg.read
        )

        self.analyzer = Analyzer(self.knowledge)
        self.planner = Planner(self.knowledge, chaser_mass)
        self.executor = Executor(forceEffector)

    def UpdateState(self, CurrentSimNanos):
        # print("MAPEK CALLED", int(CurrentSimNanos * 1e-9))
        monitoredData = self.monitor.updateState(CurrentSimNanos)

        analysisResult = self.analyzer.analyze(monitoredData)
        force, torque = self.planner.plan(analysisResult, CurrentSimNanos)

        if self.planner.docked or analysisResult.dock_ready:
            # Find a way to stop the simulation
            self.simulation.StopSimulation()

        self.executor.execute(force, torque, CurrentSimNanos)

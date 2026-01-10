from monitor import Monitor
from analyzer import Analyzer
from planner import Planner
from executor import Executor
from knowledge_base import KnowledgeBase
from model import actuation_command, analysis

from Basilisk.architecture import sysModel
from Basilisk.simulation.extForceTorque import ExtForceTorque
from Basilisk.simulation.simpleNav import SimpleNav


class MAPEK_Module(sysModel.SysModel):
    def __init__(self,
                 target: SimpleNav,
                 chaser: SimpleNav,
                 forceEffector: ExtForceTorque,
                 modelTag="autonomous"):
        super().__init__()
        self.ModelTag = modelTag

        self.monitor = Monitor()
        self.monitor.subscribeTo(
            target.transOutMsg.read,
            chaser.transOutMsg.read
        )

        self.analyzer = Analyzer()
        self.planner = Planner()
        self.executor = Executor(forceEffector)

        self.knowledge = KnowledgeBase()

    def updateState(self, CurrentSimNanos):
        self.monitor.updateState(CurrentSimNanos)

        analysisResult = self.analyzer.analyze(self.monitor)
        actuationCommand = self.planner.plan(analysisResult)

        self.executor.execute(actuationCommand, CurrentSimNanos)
        self.logStep(CurrentSimNanos, analysisResult, actuationCommand)

    def logStep(
            self,
            time,
            analysisCommand: analysis.Analysis,
            command: actuation_command.ActuationCommand
    ):
        rho = analysisCommand["relPos_H"]
        rho_dot = analysisCommand["relVel_H"]

        # TODO: Convert force to delta-v later; placeholder for now
        delta_v = command["force_H"]

        self.knowledge.log(
            time=time,
            rho=rho,
            rho_dot=rho_dot,
            delta_v=delta_v
        )

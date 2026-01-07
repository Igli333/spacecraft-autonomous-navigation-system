from Basilisk.architecture import sysModel
from .monitor import Monitor
from .knowledge_base import KnowledgeBase
from .analyzer import Analyzer
from .planner import Planner
from .executor import Executor


class MAPEK_Module(sysModel.SysModel):
    def __init__(self, target, chaser, modelTag="autonomous"):
        super().__init__()
        self.ModelTag = modelTag

        self.monitor = Monitor()
        self.monitor.subscribeTo(
            target.transOutMsg.read,
            chaser.transOutMsg.read
        )

        self.analyzer = Analyzer()
        self.planner = Planner()
        self.executor = Executor()

        self.knowledge = KnowledgeBase()

    def updateState(self, currentTime):
        self.monitor.updateState(currentTime)

        analysis = self.analyzer.analyze(self.monitor)
        plan = self.planner.plan(analysis, self.knowledge)
        self.executor.execute(plan)

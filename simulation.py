import os
import numpy as np

from Basilisk.utilities import SimulationBaseClass, macros, simIncludeGravBody, vizSupport
from Basilisk.simulation import spacecraft, simpleNav, simSynch, extForceTorque

from Basilisk.utilities import RigidBodyKinematics as rbk

from app import model, autonomousModule, utils

try:
    from Basilisk.simulation import vizInterface
except ImportError:
    pass

from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(showPlots=True, liveStream=True, broadcastStream=True):
    simulation = SimulationBaseClass.SimBaseClass()

    simulationTaskName = "simTask"
    simulationProcessName = "simProcess"
    simulationTimeStep = macros.sec2nano(1)

    dynProcess = simulation.CreateNewProcess(simulationProcessName)
    dynProcess.addTask(simulation.CreateNewTask(simulationTaskName, simulationTimeStep))

    # Defining our spacecrafts
    target = spacecraft.Spacecraft()
    target.ModelTag = "target"

    chaser = spacecraft.Spacecraft()
    chaser.ModelTag = "chaser"

    target.hub.mHub = 750.0  # kg
    target.hub.IHubPntBc_B = [[900.0, 0.0, 0.0],
                              [0.0, 800.0, 0.0],
                              [0.0, 0.0, 600.0]]

    # Chaser mass properties
    chaser.hub.mHub = 500.0  # kg
    chaser.hub.IHubPntBc_B = [[600.0, 0.0, 0.0],
                              [0.0, 550.0, 0.0],
                              [0.0, 0.0, 450.0]]

    # Initial orbital states (Target)
    target.hub.r_CN_NInit = [7000e3, 0.0, 0.0]
    target.hub.v_CN_NInit = [0.0, 7.546e3, 0.0]
    target.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    target.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

    utils.applyInitialRandomization(target, chaser, docking_port_N=None, seed=None)

    chaserForceEffect = extForceTorque.ExtForceTorque()
    chaserForceEffect.ModelTag = "chaserForce"

    chaserForceEffect.extForce_B = [0.0, 0.0, 0.0]
    # chaserForceEffect.extTorque_B = [0.0, 0.0, 0.0]

    chaser.addDynamicEffector(chaserForceEffect)

    simulation.AddModelToTask(simulationTaskName, chaserForceEffect)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravBodies = gravFactory.createBodies('sun', 'earth')
    gravBodies['earth'].isCentralBody = True

    # attach gravity model to spacecraft
    gravFactory.addBodiesTo(target)
    gravFactory.addBodiesTo(chaser)

    # setup SPICE interface for celestial objects
    timeInitString = "2002 APRIL 1 00:09:30.0"
    spiceObject = gravFactory.createSpiceInterface(time=timeInitString, epochInMsg=True)
    spiceObject.zeroBase = 'Earth'
    simulation.AddModelToTask(simulationTaskName, spiceObject)

    simulation.AddModelToTask(simulationTaskName, target)
    simulation.AddModelToTask(simulationTaskName, chaser)

    # Navigation (state access)
    targetNav = simpleNav.SimpleNav()
    targetNav.scStateInMsg.subscribeTo(target.scStateOutMsg)
    simulation.AddModelToTask(simulationTaskName, targetNav)

    chaserNav = simpleNav.SimpleNav()
    chaserNav.scStateInMsg.subscribeTo(chaser.scStateOutMsg)
    simulation.AddModelToTask(simulationTaskName, chaserNav)

    # Logging
    targetRec = targetNav.transOutMsg.recorder()
    chaserRec = chaserNav.transOutMsg.recorder()

    simulation.AddModelToTask(simulationTaskName, targetRec)
    simulation.AddModelToTask(simulationTaskName, chaserRec)

    if not vizSupport.vizFound:
        if liveStream:
            clockSync = simSynch.ClockSynch()
            clockSync.accelFactor = 15.0
            simulation.AddModelToTask(simulationTaskName, clockSync)

        viz = vizSupport.enableUnityVisualization(simulation,
                                                  simulationTaskName,
                                                  [target, chaser],
                                                  broadcastStream=broadcastStream,
                                                  liveStream=liveStream
                                                  )

        viz.reqComProtocol = "tcp"
        viz.reqComAddress = "localhost"
        viz.reqPortNumber = "5556"

        # To set broadcast port:
        viz.pubComProtocol = "tcp"
        viz.pubComAddress = "localhost"
        viz.pubPortNumber = "5570"

        viz.settings.trueTrajectoryLinesOn = -1
        viz.settings.orbitLinesOn = 2
        viz.settings.mainCameraTarget = "chaser"

    mape_k = autonomousModule.MAPEK_Module(targetNav, chaserNav, chaserForceEffect, chaser.hub.mHub)

    simulation.AddModelToTask(simulationTaskName, mape_k)

    simulation.ConfigureStopTime(macros.sec2nano(1000))  # 10 minutes
    simulation.InitializeSimulation()
    simulation.ExecuteSimulation()

    print("Target samples:", len(targetRec.r_BN_N))
    print("Chaser samples:", len(chaserRec.r_BN_N))

    # --- Post-processing ---
    rT = np.vstack(targetRec.r_BN_N)
    rC = np.vstack(chaserRec.r_BN_N)
    relPos = rC - rT

    print(rT)
    print(rC)

    if showPlots:
        utils.animateChaserTarget(rC, rT)

    return relPos


if __name__ == "__main__":
    run(
        True,  # show_plots
        True,
        True
    )

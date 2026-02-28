import os
import json
import sys
from pathlib import Path

import numpy as np
import autonomousModule, sim_utils

from Basilisk.utilities import SimulationBaseClass, macros, simIncludeGravBody, vizSupport
from Basilisk.simulation import spacecraft, simpleNav, simSynch, extForceTorque

try:
    from Basilisk.simulation import vizInterface
except ImportError:
    pass

from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(showPlots=True, liveStream=True, broadcastStream=True, configuration='earth-docking.json'):
    with open(f"{Path(__file__).resolve().parent}/config/{configuration}") as f:
        config = json.load(f)

    simulationConfig = config["simulation"]
    spiceConfig = simulationConfig["spice"]
    vizConfig = simulationConfig["visualization"]

    simulation = SimulationBaseClass.SimBaseClass()

    simulationTaskName = "simTask"
    simulationProcessName = "simProcess"
    simulationTimeStep = macros.sec2nano(simulationConfig['step_sec'])

    dynProcess = simulation.CreateNewProcess(simulationProcessName)
    dynProcess.addTask(simulation.CreateNewTask(simulationTaskName, simulationTimeStep))

    target = spacecraft.Spacecraft()
    target.ModelTag = "target"

    chaser = spacecraft.Spacecraft()
    chaser.ModelTag = "chaser"

    target.hub.mHub = simulationConfig['target']['mass']
    target.hub.IHubPntBc_B = np.array(
        simulationConfig['target']['inertia']
    )

    chaser.hub.mHub = simulationConfig['chaser']['mass']
    chaser.hub.IHubPntBc_B = np.array(
        simulationConfig['chaser']['inertia']
    )

    target.hub.r_CN_NInit = simulationConfig['target']['position_m']
    target.hub.v_CN_NInit = simulationConfig['target']['velocity_m_s']

    target.hub.sigma_BNInit = [[x] for x in simulationConfig['target']['sigma']]
    target.hub.omega_BN_BInit = [[x] for x in simulationConfig['target']['omega_rad_s']]

    seed = simulationConfig.get("seed", None)
    sim_utils.applyInitialRandomization(target, chaser, docking_port_N=None, seed=seed)

    chaserForceEffect = extForceTorque.ExtForceTorque()
    chaserForceEffect.ModelTag = "chaserForce"

    chaserForceEffect.extForce_B = [0.0, 0.0, 0.0]
    chaserForceEffect.extTorquePntB_B = [0.0, 0.0, 0.0]

    chaser.addDynamicEffector(chaserForceEffect)
    simulation.AddModelToTask(simulationTaskName, chaserForceEffect)

    body = simulationConfig.get('body', 'earth')
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravBodies = gravFactory.createBodies('sun', body)
    gravBodies[body].isCentralBody = True

    gravFactory.addBodiesTo(target)
    gravFactory.addBodiesTo(chaser)

    spiceObject = gravFactory.createSpiceInterface(time=spiceConfig["epoch"], epochInMsg=True)
    spiceObject.zeroBase = spiceConfig["zero_base"]
    simulation.AddModelToTask(simulationTaskName, spiceObject)

    simulation.AddModelToTask(simulationTaskName, target)
    simulation.AddModelToTask(simulationTaskName, chaser)

    targetNav = simpleNav.SimpleNav()
    targetNav.scStateInMsg.subscribeTo(target.scStateOutMsg)
    simulation.AddModelToTask(simulationTaskName, targetNav)

    chaserNav = simpleNav.SimpleNav()
    chaserNav.scStateInMsg.subscribeTo(chaser.scStateOutMsg)
    simulation.AddModelToTask(simulationTaskName, chaserNav)

    targetRec = targetNav.transOutMsg.recorder()
    chaserRec = chaserNav.transOutMsg.recorder()

    simulation.AddModelToTask(simulationTaskName, targetRec)
    simulation.AddModelToTask(simulationTaskName, chaserRec)

    if vizSupport.vizFound:
        if liveStream:
            clockSync = simSynch.ClockSynch()
            clockSync.accelFactor = vizConfig["clock_acceleration_factor"]
            simulation.AddModelToTask(simulationTaskName, clockSync)

        viz = vizSupport.enableUnityVisualization(
            simulation,
            simulationTaskName,
            [target, chaser],
            broadcastStream=broadcastStream,
            liveStream=liveStream
        )

        viz.reqComProtocol = "tcp"
        viz.reqComAddress = vizConfig["request"]["address"]
        viz.reqPortNumber = str(vizConfig["request"]["port"])

        viz.pubComProtocol = "tcp"
        viz.pubComAddress = vizConfig["publish"]["address"]
        viz.pubPortNumber = str(vizConfig["publish"]["port"])

        viz.settings.trueTrajectoryLinesOn = -1
        viz.settings.orbitLinesOn = 2
        viz.settings.mainCameraTarget = "target"

    mape_k = autonomousModule.MAPEK_Module(targetNav, chaserNav, chaserForceEffect, config)

    simulation.AddModelToTask(simulationTaskName, mape_k)

    simulation.ConfigureStopTime(macros.sec2nano(simulationConfig['stop_time_sec']))
    simulation.InitializeSimulation()
    simulation.ExecuteSimulation()

    print("Target samples:", len(targetRec.r_BN_N))
    print("Chaser samples:", len(chaserRec.r_BN_N))

    # Post-processing
    rT = np.vstack(targetRec.r_BN_N)
    rC = np.vstack(chaserRec.r_BN_N)
    relPos = rC - rT

    if showPlots:
        sim_utils.animateChaserTarget(rC, rT)

    return relPos


if __name__ == "__main__":
    run(
        False,  # show_plots
        True,
        True,
        sys.argv[1]
    )

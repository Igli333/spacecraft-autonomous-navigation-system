import os
import json
import numpy as np
import autonomousModule, sim_utils

from Basilisk.utilities import SimulationBaseClass, macros, simIncludeGravBody, vizSupport
from Basilisk.simulation import spacecraft, simpleNav, simSynch, extForceTorque

from dotenv import load_dotenv

try:
    from Basilisk.simulation import vizInterface
except ImportError:
    pass

from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

load_dotenv("spacecraft-navigation-system/sim.env")


def run(showPlots=True, liveStream=True, broadcastStream=True):
    simulation = SimulationBaseClass.SimBaseClass()

    simulationTaskName = "simTask"
    simulationProcessName = "simProcess"
    simulationTimeStep = macros.sec2nano(float(os.getenv("SIM_TIME_STEP_SEC", 1.0)))

    dynProcess = simulation.CreateNewProcess(simulationProcessName)
    dynProcess.addTask(simulation.CreateNewTask(simulationTaskName, simulationTimeStep))

    # Defining our spacecrafts
    target = spacecraft.Spacecraft()
    target.ModelTag = "target"

    chaser = spacecraft.Spacecraft()
    chaser.ModelTag = "chaser"

    target.hub.mHub = float(os.getenv("TARGET_MASS", 1750.0))
    target.hub.IHubPntBc_B = np.array(
        json.loads(os.getenv("TARGET_INERTIA"))
    )

    # [[1000.0, 0.0, 0.0],
    #  [0.0, 1000.0, 0.0],
    #  [0.0, 0.0, 1000.0]]

    # Chaser mass properties
    chaser.hub.mHub = float(os.getenv("CHASER_MASS", 500.0))
    chaser.hub.IHubPntBc_B = np.array(
        json.loads(os.getenv("CHASER_INERTIA"))
    )

    # [[600.0, 0.0, 0.0],
    #   [0.0, 550.0, 0.0],
    #   [0.0, 0.0, 450.0]]

    # Initial orbital states (Target)
    target.hub.r_CN_NInit = [
        float(os.getenv("R_INIT_X", 7000e3)),
        float(os.getenv("R_INIT_Y", 0.0)),
        float(os.getenv("R_INIT_Z", 0.0))
    ]
    target.hub.v_CN_NInit = [
        float(os.getenv("V_INIT_X", 0.0)),
        float(os.getenv("V_INIT_Y", 7.546e3)),
        float(os.getenv("V_INIT_Z", 0.0))
    ]
    target.hub.sigma_BNInit = [
        [float(os.getenv("SIGMA_INIT_X", 0.0))],
        [float(os.getenv("SIGMA_INIT_Y", 0.0))],
        [float(os.getenv("SIGMA_INIT_Z", 0.0))]
    ]
    target.hub.omega_BN_BInit = [
        [float(os.getenv("OMEGA_INIT_X", 0.0))],
        [float(os.getenv("OMEGA_INIT_Y", 0.0))],
        [float(os.getenv("OMEGA_INIT_Z", 0.0))]
    ]

    seed = os.getenv("RANDOM_SEED")
    if seed:
        seed = int(seed)
    else:
        seed = None
    sim_utils.applyInitialRandomization(target, chaser, docking_port_N=None, seed=seed)

    chaserForceEffect = extForceTorque.ExtForceTorque()
    chaserForceEffect.ModelTag = "chaserForce"

    chaserForceEffect.extForce_B = [0.0, 0.0, 0.0]
    chaserForceEffect.extTorquePntB_B = [0.0, 0.0, 0.0]

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
    timeInitString = os.getenv("SPICE_EPOCH", "2002 APRIL 1 00:09:30.0")
    spiceObject = gravFactory.createSpiceInterface(time=timeInitString, epochInMsg=True)
    spiceObject.zeroBase = os.getenv("SPICE_ZERO_BASE", "Earth")
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

    if vizSupport.vizFound:
        if liveStream:
            clockSync = simSynch.ClockSynch()
            clockSync.accelFactor = float(os.getenv("CLOCK_ACCEL_FACTOR", 100.0))
            simulation.AddModelToTask(simulationTaskName, clockSync)

        viz = vizSupport.enableUnityVisualization(
            simulation,
            simulationTaskName,
            [target, chaser],
            broadcastStream=broadcastStream,
            liveStream=liveStream
        )

        viz.reqComProtocol = "tcp"
        viz.reqComAddress = os.getenv("VIZ_REQ_ADDR", "localhost")
        viz.reqPortNumber = os.getenv("VIZ_REQ_PORT", "5556")

        # To set broadcast port:
        viz.pubComProtocol = "tcp"
        viz.pubComAddress = os.getenv("VIZ_PUB_ADDR", "localhost")
        viz.pubPortNumber = os.getenv("VIZ_PUB_PORT", "5570")

        viz.settings.trueTrajectoryLinesOn = -1
        viz.settings.orbitLinesOn = 2
        viz.settings.mainCameraTarget = "target"

    mape_k = autonomousModule.MAPEK_Module(targetNav, chaserNav, chaserForceEffect)

    simulation.AddModelToTask(simulationTaskName, mape_k)

    simulation.ConfigureStopTime(macros.sec2nano(float(os.getenv("SIM_STOP_TIME_SEC", 1500.0))))
    simulation.InitializeSimulation()
    simulation.ExecuteSimulation()

    print("Target samples:", len(targetRec.r_BN_N))
    print("Chaser samples:", len(chaserRec.r_BN_N))

    # --- Post-processing ---
    rT = np.vstack(targetRec.r_BN_N)
    rC = np.vstack(chaserRec.r_BN_N)
    relPos = rC - rT

    if showPlots:
        sim_utils.animateChaserTarget(rC, rT)

    return relPos


if __name__ == "__main__":
    run(
        True,  # show_plots
        True,
        True
    )

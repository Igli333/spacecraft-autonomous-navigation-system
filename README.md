# Autonomous Navigation for Docking Spacecraft

### Implemented on the [Basilisk](https://github.com/AVSLab/basilisk) simulator

#### To run
1. Download Basilisk and build it on your local machine
2. Clone this repository inside of ./basilisk/
    - `cd` into the folder and run `docker compose up --build`
    - If needed, you can change all system variables inside `sim.env`.
    - Go back using `cd ..`
3. Run `python ./spacecraft-autonomous-navigation-system/simulation.py <config-file-name>.json`

#### To show the 3D visualization:
1. Download [Vizard](https://avslab.github.io/basilisk/Vizard/VizardDownload.html)
2. Run the simulation as in point 3
3. Connect using `tcp://localhost:5556` 

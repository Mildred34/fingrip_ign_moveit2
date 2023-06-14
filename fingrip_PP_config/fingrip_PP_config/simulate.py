# Make sure to have CoppeliaSim running, with followig scene loaded:
#
# scenes/messaging/.ttt
#
# Do not launch simulation, then run this script
from utilities.simulation import Simulation as simx

print('Program started')

simulator = simx()

# Start simulation:
simulator.startSimulation()

while (t := simulator.sim.getSimulationTime()):
    try:
        simulator.simStep()
    except KeyboardInterrupt:
        print('You pressed Ctrl+C!')
        break

# Stop simulation
simulator.end()

# Wait until above movement sequence finished executing:
# simulator.waitForMovementExecuted('up')

SINGLE SIMULATION
Use virtual_sim.py to launch the simulation without ROS in the loop
Copy all the files to have tactical behaviour in this folder (vrtual_sim)
Adjust te parameters.py that is in this folder in case of need,
But parameters can also be directly passed to tactical behaviour from the script without the need to use parameters.py
The script shows the simulation with an animated plot in matplotlib.
The result is a json file. Use parse_sim_result.py to plot the result.

SIMULATING MULTIPLE parameters
Use the file sim_find_params.py to simulate multiple parameters at once.
It is a copy paste with modifcations of the virtual_sim.py script.
Set the parameters to simulate in the main().
The resulting json is found in "/home/gianfi/Documents/KvaserCar_outdoor_test/" the project root.
I moved the old results in "test_tactical_node/virtual_sim/results_virtual_sim/find_parameters".
To plot use parse_sim_result_exploring_assumptions.py
It produced a "faceted" plot 


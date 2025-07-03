SIMULATION WITH ROS IN THE LOOP

To plot use plot_data.py

------------------ 1st TERMINAL ----------------------
***build the nodes
from pkg root
cd /home/gianfi/Documents/KvaserCar_outdoor_test/
source /opt/ros/humblr/setup.bash
colcon build --packages-select tactical_msgs tactical_node obops_receiver
source install/setup.bash

***launch obops_receiver
ros2 run obps_receiver obps_receiver

***launch tactical node
ros2 run tactical_node tactical_node

--------------------- 2nd  TERMINAL ----------------------
***plotter node
cd /home/gianfi/Documents/KvaserCar_outdoor_test/
source /opt/ros/humblr/setup.bash
source install/setup.bash
python3 /home/gianfi/Documents/KvaserCar_outdoor_test/test_tactical_node/plotter_node.py


--------------------- 3d  TERMINAL ----------------------
cd /home/gianfi/Documents/KvaserCar_outdoor_test/
source /opt/ros/humblr/setup.bash
source install/setup.bash
python3 /home/gianfi/Documents/KvaserCar_outdoor_test/test_tactical_node/sim_node.py



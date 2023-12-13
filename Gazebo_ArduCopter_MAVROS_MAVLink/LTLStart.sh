#!/bin/bash

cd ~/catkin_ws/ 
catkin build
source ~/.bashrc
cd ~

cp ~/Drone_LTL_Planner/build/pathGrid.txt ~/catkin_ws/src/iq_gnc/

# Command 1
gnome-terminal --tab --title="GazeboSim" -- bash -c "roslaunch iq_sim LTLForest.launch; exec bash"

sleep 5
 
# Command 2
gnome-terminal --tab --title="MAVLink" -- bash -c "./startsitl.sh; exec bash"

sleep 5

# Command 3
gnome-terminal --tab --title="MAVRos" -- bash -c "roslaunch iq_sim apm.launch; exec bash"

sleep 5

# Command 4
gnome-terminal --tab --title="GNC Script" -- bash -c "rosrun iq_gnc LTLForest; exec bash"

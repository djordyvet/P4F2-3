#!/bin/bash

#roscore
cd niryo_robot_ws
source devel/setup.bash
roscore

#Vision
x-terminal-emulator -e bash -c "cd niryo_robot_ws; source devel/setup.bash;roslaunch P4F2-3 nn_detector.launch;"
x-terminal-emulator -e bash -c "roslaunch my_niryo_robot_bringup calibrate.launch; ;"

#Manipulator
x-terminal-emulator -e bash -c "cd niryo_robot_ws; source devel/setup.bash;roslaunch my_niryo_robot_bringup calibrate.launch; ;"

#HMI
x-terminal-emulator -e bash -c "cd niryo_robot_ws; source devel/setup.bash;cd niryo_robot_ws/src/P4F2-3/HMI_software; python HMI_2.py;"


#nodes
x-terminal-emulator -e bash -c "cd niryo_robot_ws; source devel/setup.bash;rostopic echo /robot_current_pose;"

# Open a new terminal using x-terminal-emulator and execute commands
x-terminal-emulator -e bash -c "echo 'Hello from the new terminal'; ls;"


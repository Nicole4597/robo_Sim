#!/bin/bash

source /opt/ros/noetic/setup.bash   # setup ros

if [ "$1" = "--recompile" ]
then
    catkin_make clean
    catkin_make
else
    echo 'If you want to recompile the code launch this script with "--recompile"'
fi

source devel/setup.bash     # setup packages

# launch nodes
gnome-terminal -- roslaunch main_controller main_controller.launch

sleep 5    # wait for roscore to start

gnome-terminal -- roslaunch triskar triskarone_slam.launch
gnome-terminal -- roslaunch triskar_navigation move_base.launch

# launch services
cd src/main_controller/src
gnome-terminal -- rosrun main_controller ai_server.py
gnome-terminal -- rosrun main_controller bm_server.py
gnome-terminal -- rosrun main_controller reaction_server.py

# launch parameter reconfiguration gui
# gnome-terminal -- rosrun rqt_reconfigure rqt_reconfigure

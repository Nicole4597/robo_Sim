source /opt/ros/noetic/setup.bash   # setup ros
source ../devel/setup.bash     # setup packages
cd ../src/main_controller/src
rosrun main_controller ai_server.py    # launch node

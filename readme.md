# Instructions

this instructions assume you have moved the entire repo on the robot (e.g. using _FileZilla_)

- [Instructions](#instructions)
  - [Check Parameters](#check-parameters)
  - [Compile](#compile)
    - [ROS1 nodes](#ros1-nodes)
    - [ROS2 nodes](#ros2-nodes)
    - [Check bridge installation](#check-bridge-installation)
  - [Launching ROS1 nodes all together](#launching-ros1-nodes-all-together)
  - [Launching ROS1 nodes separately](#launching-ros1-nodes-separately)
    - [Main Controller](#main-controller)
    - [Triskarone](#triskarone)
    - [Move Base](#move-base)
    - [Behaviour Server](#behaviour-server)
    - [Actor Info Server](#actor-info-server)
    - [Reaction Server](#reaction-server)
  - [Launching ROS BRIDGE](#launching-ros-bridge)
  - [Launching ROS2 nodes](#launching-ros2-nodes)

---

## Check Parameters

before launching anything check *main_controller* parameters in ```/ros1_ws/src/main_controller/params/main_controller_params.yaml```

---

## Compile

### ROS1 nodes

    cd ros1_ws
    source /opt/ros/noetic/setup.bash
    catkin_make

### ROS2 nodes

    cd ros2_ws
    source /opt/ros/foxy/setup.bash
    colcon build

### Check bridge installation

    sudo apt-get install ros-foxy-ros1-bridge

---

## Launching ROS1 nodes all together

N.B. launching nodes separately facilitates debugging

1. launch triskarone and move base, in two terminals:

        cd ros1_ws
        source /opt/ros/noetic/setup.bash
        source devel/setup.bash
        roslaunch triskar triskarone_slam.launch


        cd ros1_ws
        source /opt/ros/noetic/setup.bash
        source devel/setup.bash
        roslaunch triskar_navigation move_base.launch

2. launch custom nodes, in another terminal

        cd ros1_ws
        source /opt/ros/noetic/setup.bash
        source devel/setup.bash
        roslaunch main_controller chiroli.launch

3. launching rqt-reconfigure (parameters reconfiguration)

        source /opt/ros/noetic/setup.bash
        rosrun rqt_reconfigure rqt_reconfigure


---

## Launching ROS1 nodes separately

N.B. check out: ```ros1_ws/help_scripts/```

### Main Controller

open a new terminal, ```cd``` to this directory and execute:

    cd ros1_ws
    source /opt/ros/noetic/setup.bash
    source devel/setup.bash
    roslaunch main_controller main_controller.launch

### Triskarone

open a new terminal, ```cd``` to this directory and execute:

    cd ros1_ws
    source /opt/ros/noetic/setup.bash
    source devel/setup.bash
    roslaunch triskar triskarone_slam.launch

### Move Base

open a new terminal, ```cd``` to this directory and execute:

    cd ros1_ws
    source /opt/ros/noetic/setup.bash
    source devel/setup.bash
    roslaunch triskar_navigation move_base.launch

### Behaviour Server

open a new terminal, ```cd``` to this directory and execute:

    cd ros1_ws
    source /opt/ros/noetic/setup.bash
    source devel/setup.bash
    cd src/main_controller/src
    rosrun main_controller bm_server.py

### Actor Info Server

open a new terminal, ```cd``` to this directory and execute:

    cd ros1_ws
    source /opt/ros/noetic/setup.bash
    source devel/setup.bash
    cd src/main_controller/src
    rosrun main_controller ai_server.py

### Reaction Server

open a new terminal, ```cd``` to this directory and execute:

    cd ros1_ws
    source /opt/ros/noetic/setup.bash
    source devel/setup.bash
    cd src/main_controller/src
    rosrun main_controller reaction_server.py

---

## Launching ROS BRIDGE

open a new terminal and execute:

    source /opt/ros/noetic/setup.bash
    source /opt/ros/foxy/setup.bash
    ros2 run ros1_bridge dynamic_bridge --bridge-all-topics

notice that the bridge requires ```roscore```, hence a ros1 node must be already running (e.g. _main_controller_)

---

## Launching ROS2 nodes

open a new terminal, ```cd``` to this directory and execute:

    source /opt/ros/foxy/setup.bash
    source install/setup.bash
    ros2 run py_pub my_node

this last node is the interface with Lorenzo's work: it listens for messages in ```/actor_position``` (my_interfaces/ActorInfo.msg) and ```/scenic_action``` (std_msgs/String)

---

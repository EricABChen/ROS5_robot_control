# ecse473_f23_axc1293_navigation_control

## Package description
This repo is a ROS package that contains the navigation control simulation for a robot to avoid collision

## Dependency:
In order to run/view this model, the listed dependencies MUST be satisfied:
-  ROS(Noetic)
-  ROS map server
-  STDR simulator package

## Roslaunch with launch file
To launch the model in STDR, you need to first
- Download this package into your catkin workspace `~/catkin_ws/src/`
- Build the package with `catkin_make`
- Source: `source catkin_ws/devel/setup.bash`

### Launch with STDR and a model

`roslaunch robot_no_crash stdr_launcher.launch`

This command will launch the STDR model with a default robot

### Connect to multiple robot models

- Create a new robot Node with default nodename:
 
 `roslaunch robot_no_crash robot_supervisor.launch`
 
- Create multiple robot name under different namespace

`rosrun robot_no_crash robot_no_crash_node __ns:=robot1`

- Launch STDR simulator

`roslaunch stdr_launchers server_with_map_and_gui.launch`
 
- Launch rqt_gui

`rosrun rqt_gui rqt_gui`

- Add a new robot with a sensor and place it on the map

- The robot is connected automatically

Note: you can also manually add a node by `rosrun robot_no_crash robot_no_crash_node __ns:=robot0`, make sure that `__ns` should match with the STDR simulator

# final_project

Requirements : Ubuntu 20.0.4

Please follow the meeting for the setup of the project and clone these files.

Create a catkin workspace named catkin_ws in home directory:
Launch cmds:

source ~/.bashrc
catkin_make
source ~/catkin_ws/source/devel/setup.bash

To view the robot in gazebo turtlebot3_world:
roslaunch vehicle_description Main.launch

To launch the navigation stack:
roslaunch vehicle_navigation navigation_main.launch

Note:
A template for global planner has been added. You can find the files under global_planner/plugins.
This is a test global planner and it is implemented in navigation_stack.

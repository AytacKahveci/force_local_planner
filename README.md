# Force Local Planner
This ROS package contains a local planner implementation for differential drive robots. By calculating attractive and the repulsive forces acting on the robot,
planner computes local plan of the robot at each control cycle. </br> 
The implemented algorithm can be found in this [academic paper](https://ieeexplore.ieee.org/document/655062/references#references).

Local planner is created as a nav_core plugin and it can be directly used with move_base. Planner is tested on the Gazebo simulation with Tiago robot. </br>

[![TIAGo Gazebo Simulation](http://img.youtube.com/vi/22neL_Uo6p8/0.jpg)](http://www.youtube.com/watch?v=22neL_Uo6p8 "Force Local Planner")

May the force be with you :)
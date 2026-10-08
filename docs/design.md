# Design 

## Things to decide

1. Name: I propose "EasyNavigation"
2. License: GPLv3
  1. Pro: It lets to include Apache 2.0 and BSD
  2. Pro: It lets to create a bussines model based on selling versions with privative license
  3. Contra: Some people could have concerns to use the software
  4. Fact: We should mantain the entire copyright, so we should inform in each PR that the full copyright is ours

## Requisites and characteristics

1. Easy to use
2. Well sincronized, with real time and non-real time parts
  1. All in one process 
3. Use a TUI, like (this)[https://roscon.ros.org/2024/talks/r2s-_A_Terminal_User_Interface_for_ROS_2.pdf]
4. Independent from the space representation, in any way.
  1. Backward compatibility with costmap
  2. Use of others like gridmaps or costmaps
  3. Multilvel support
5. Tool for easy calibration of robot parameters
6. Mechanisms to use Nav2 plugins with something lihe a bridge library
7. Clear separatio of Navigation logic, with no ROS code, and ROS code, only for comunication
8. Use of topics instead (or apart) of actions to send goals and receive feedback
9. Use of other good simulator instead of Gazebo (which?)
  1. Open3D?
  2. Unity
  3. ...
10. API documentation with doxygen
11. Tests for all the code from the first lines of code

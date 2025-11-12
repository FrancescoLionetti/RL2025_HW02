# 🤖 Robotics Lab - Homework 2: Control your robot
 
## Introduction
This repository contains the solution for Homework 2 of the Robotics Lab class. The goal of the work is to develop and implement **kinematic** and **vision-based** controllers for a KUKA iiwa robotic manipulator arm, using the ROS 2 packages `ros2_kdl_package` and `ros2_iiwa`.
 
The source code and documentation are organized to solve the following problems:
1.**Kinematic Control:** Implementation of a velocity control with a null-space term for joint limit avoidance andd conversion of the trajectory logic into a ROS 2 Action Server
2.**Vision-based Control:** Development of a controller for a *look-at-point* task using pose data from an ArUco tag.
 
---
 
## 🛠️ Prerequisites
 
* **ROS 2 (Humble):** Configured working environment.
* **Base Packages:**
    * `ros2_kdl_package` (forked and modified)
    * `ros2_iiwa` (forked and modified to include the ArUco model)
    * `aruco_ros` (for marker detection) 
* **Specific Dependencies:**
    * **KDL:** Used for kinematic calculations (forward, inverse kinematics, and Jacobian).
    * **ROS 2 Parameter Bridge:** Necessary for the Gazebo position update service.
 
## ⚙️ Getting Started
 
To set up and test the project, follow these steps inside your ROS 2 workspace
 
1.  **Clone the Repository:**
    ```shell
    $ cd ~/ros2_ws/src
    $ git clone https://github.com/FrancescoLionetti/RL2025_HW02.git
    ```
 
2.  **Build and Source:**
    ```shell
     $	cd ..
     $	colcon build
     $	source install/setup.bash
    ```
 
## 🏃 Execution Instructions (Kinematic Control)
 
This section details how to build and run the Kinematic Control solution.
 
## **1A. Parameterized launch and kinematic control execution**
In the first terminal launch the commands to start RViz:
```shell
$ ros2 launch iiwa_bringup iiwa.launch.py
```
In another terminal launch the following command:
 
```shell
$ ros2 launch ros2_kdl_package kdl_node.launch.py
```
By default the node publishes joint position commands. To use the velocity commands see the point 1B.
 
## **1B. Simulation of the Kuka IIWA robot in RViz with the choice of the controller**
In the first terminal launch the commands to start RViz:
```shell
$ ros2 launch iiwa_bringup iiwa.launch.py coomand_interface:="velocity" robot_controller:="velocity_command"
```
In a second terminal launch the following command for the velocity control:
 
```shell
$	ros2 launch ros2_kdl_package kdl_node.launch.py cmd_interface:=velocity ctrl:=<type>
```
where < type > can be 'velocity_ctrl' or 'velocity_ctrl_null' (to implement a controller that avoids joint limits)
 
## **1C. Simulation of the Kuka IIWA robot in RViz with the Action Client structure**
To simulate with action-client service, after you launched with velocity command, in another terminal you have to launch:
 
```shell
$	ros2 launch ros2_kdl_package kdl_action.launch.py 
```
And then:
 
```shell
$	ros2 run ros2_kdl_package action_client_node <x> <y> <z> 
```
 
## 🏃 Vision Based Control
 
This section details how to build and run the Vision Based Control solution.
 
## **Detect an ArucoTag in a Gazebo world**
In the first terminal launch the command to visualize the robot in Gazebo world:
```shell
  $ ros2 launch ros2_kdl_package gaz.launch.py
```
Then launch:
 
```shell
$	ros2 launch aruco_ros single.launch.py marker_size:=0.1 marker_id:=18
```
Switch the topic of image_view.
 
## **Loop-at-point task with vision-control-based**
 
If you want activate a vision-control based put this command:
 
```shell
$ ros2 run ros2_kdl_package ros2_vision_control_node cmd_interface:=velocity ctrl:=vision
```
To switch the auco tag position whit CLI launch this command:
 
```shell
$ ros2 service call /world/default/set_pose ros_gz_interfaces/srv/SetEntityPose 	"{entity: {name: 'aruco_tag', type: 1}, pose: {position: {x: 1.3, y: 0.6, z: 0.2}, 	orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```
you can set different position and orientation.

# artificial_body
This repository contains perception, planning and control models of an Autonomous Mobile Robot equipped with a Robotic Arm. 

## Installation of Open Manipulator X on Ubuntu 16.04 with ROS Kinetic and Gazebo 7
0. Install Docker https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-20-04-pt
1. Install Luffer https://github.com/aforechi/luffer.git
2. Install ROS Kinetic https://github.com/aforechi/luffer/blob/master/docs/ROS.md
3. Compile ROS Kinetic packages https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/
4. Run Gazebo Simulation https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_simulation/
5. Run Robotis Manipulation https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_controller_package/

## Installation of Drake Toolbox (https://drake.mit.edu)
1. https://manipulation.csail.mit.edu/drake.html

# Manipulation Demo
0. Install Blender sudo apt install blender
1. Export Open Manipulator X to URDF format
```
cd ~/catkin_ws/src/open_manipulator/open_manipulator_description/urdf
rosrun xacro xacro open_manipulator.urdf.xacro > open_manipulator.urdf
cd ~/artificial_body/scripts && ./convert_stl_to_blend.sh ~/catkin_ws/src/open_manipulator/open_manipulator_description/meshes

sudo mkdir -p /opt/drake/share/drake/manipulation/models/open_manipulator/urdf
sudo cp ~/catkin_ws/src/open_manipulator/open_manipulator_description/urdf/open_manipulator.urdf /opt/drake/share/drake/manipulation/models/open_manipulator/urdf
sudo cp -r ~/catkin_ws/src/open_manipulator/open_manipulator_description/meshes /opt/drake/share/drake/manipulation/models/open_manipulator

```
5. Run demo.ipynb

```
cd ~/artificial_body/notebook
drake-visualizer&
jupyter notebook 
```

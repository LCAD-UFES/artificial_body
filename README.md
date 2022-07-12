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
1. Install Drake from binary https://drake.mit.edu/from_binary.html
```
cd ~
wget https://github.com/RobotLocomotion/drake/releases/download/v1.1.0/drake-20220328-mac.tar.gz
mkdir -p env/drake
tar -xvzf drake-20220328-mac.tar.gz -C env/drake --strip-components=1
python3 -m venv env/drake --system-site-packages
source env/drake/bin/activate
env/drake/share/drake/setup/install_prereqs
```
1.1 Config firewall according to http://lcm-proj.github.io/multicast_setup.html

2. Run the particles example
```
git clone https://github.com/RobotLocomotion/drake-external-examples.git
cd drake-external-examples
mkdir drake_cmake_installed-build && cd drake_cmake_installed-build
cmake -DCMAKE_PREFIX_PATH=~/env/drake ../drake_cmake_installed
make

env/drake/bin/drake-visualizer &
(cd src/particles && exec ./uniformly_accelerated_particle)
```

3. Run the kuka iiwa example
```
cd ~/artificial_body
mkdir build && cd build && cmake .. && make && cd ..
env/drake/bin/drake-visualizer &
./build/src/kuka_iiwa_arm/kuka_simulation &
./build/src/kuka_iiwa_arm/kuka_plan_runner &
./build/src/kuka_iiwa_arm/move_iiwa_ee -x 0.8 -y 0.3 -z 0.25 -yaw 1.57
```
# Manipulation Demo
0. Install Blender sudo apt install blender
1. Export Open Manipulator X to URDF format
```
cd ~/catkin_ws/src/open_manipulator/open_manipulator_description/urdf
rosrun xacro xacro open_manipulator.urdf.xacro > open_manipulator.urdf
cd ~/artificial_body/scripts && ./convert_stl_to_blend.sh ~/catkin_ws/src/open_manipulator/open_manipulator_description/meshes
gz sdf -p open_manipulator.urdf > open_manipulator.sdf
```
5. Run demo.ipynb

```
cd ~/artificial_body/notebooks
drake-visualizer& or bazel run //tools:meldis -- --open-window &
jupyter notebook 
```

## Installation of OpenCR
0. https://emanual.robotis.com/docs/en/parts/controller/opencr10/#arduino-ide
1. https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/#communication-interface
2. https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide_basic_operation/

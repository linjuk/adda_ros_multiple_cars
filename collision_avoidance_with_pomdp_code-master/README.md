## Collision Avoidance in Uncertain Environments for Autonomous Vehicles using POMDPs
### 1. Introduction
This repository contains all necessary ROS packages for building and running the simulation of the toy example. It uses the ROS [Navigation stack](http://wiki.ros.org/navigation) for setting up a map and navigating through the map.

**Note:** Currently working only on ROS Kinetic!

### 2. How to use
Build the repository using `ias make`. If you encounted an error message telling you that "ActionObservation.h" couldn't be found, create an empty file in the despot folder named "CATKIN_IGNORE" and make the project again. After it is done, delete the file and re-make again.

Then, fire up a console and type
``` 
roslaunch car_model car.launch 
```

 rViz should pop up and you can see the map loaded with the URDF model of the car. To publish a point for the path planner, use the **2D Nav Goal** button in the toolbar menu at the top.


### 3. Dependencies
In order to build all the pacakges in the navigation stack, the following dependencies have to be installed via `apt-get install`:
```
sudo apt-get install libbullet-dev libsdl-image1.2-dev
```
Furthermore, the following ROS packages are needed (if not yet installed):
```
sudo apt-get install ros-kinetic-move-base-msgs ros-kinetic-rotate-recovery ros-kinetic-costmap-converter ros-kinetic-libg2o libsuitesparse-dev
```
or just install all dependencies at once &mdash; if there are more &mdash; with
```
rosdep install --from-paths PATH_TO_WORKSPACE --ignore-src --rosdistro=kinetic
```
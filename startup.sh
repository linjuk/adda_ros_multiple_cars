#!/usr/bin/env bash

gnome-terminal -x sh -c 'roslaunch pomdp_car_launch car2.launch; exec bash'
sleep 5
gnome-terminal -x sh -c 'rosparam set joy_node/dev "/dev/input/js1"; rosrun joy joy_node; exec bash'
sleep 5
gnome-terminal -x sh -c 'roslaunch car_teleop car_teleop.launch; exec bash'


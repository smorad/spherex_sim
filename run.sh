#!/bin/bash

export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:"$(pwd)/models"
cd build
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:"$(pwd)"

# Gazebo only
gazebo --verbose ../spherex.world

# Gazebo + ROS
#source /opt/ros/kinetic/setup.bash
#roslaunch gazebo_worlds ../spherex.world

#!/bin/bash

source /opt/ros/kinetic/setup.bash
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:"$(pwd)/models"
cd build
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:"$(pwd)"

if [ $(pgrep roscore) ] ; then
    echo 'Roscore already running, not starting...'
else
    roscore &
fi

#rosrun gazebo_ros gazebo
gazebo --verbose ../spherex.world




#!/bin/bash

export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:"$(pwd)/models"
cd build
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:"$(pwd)"

if [ $DEBUG ] ; then
    gdb -ex=run --args gazebo --verbose ../spherex.world
else
    gazebo --verbose ../spherex.world
fi

#!/bin/bash

# Setup ROS home directory
#export ROS_HOME=/extern/roshome
export USE_ROSBAG=0

#Vehicle variables
export VEHICLE=pladypos
export MODEL=`rospack find snippets`/data/models/pladypos.yaml
export ALLOCATION_MODEL=`rospack find snippets`/data/allocations/x_horizontal.yaml

#Input
export JOYSTICK=/dev/input/js1
export USE_JOY=0

#Simulation variables
export IS_SIM=1
export USE_NOISE=1
export USE_VISUALIZATION=0

#Location for simulation or predefined position
export LOCATION=jarun
export USE_LOCAL_FIX=1

#Control configuration
export USE_IDENTIFICATION=1
export USE_MC=0
export USE_MULTIMASTER=0

#Frame description
export USE_TF_PREFIX=0
export TF_PREFIX=pladypos

#USBL variables
export USE_USBL_MANAGER=0
export USE_USBL=0







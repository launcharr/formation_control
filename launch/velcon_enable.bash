#!/bin/bash

rosservice call /vehicle1/ConfigureVelocityController "ControllerName: ''
desired_mode: [2, 2, 2, 2, 2, 2]"

rosservice call /vehicle2/ConfigureVelocityController "ControllerName: ''
desired_mode: [2, 2, 2, 2, 2, 2]"

rosservice call /vehicle3/ConfigureVelocityController "ControllerName: ''
desired_mode: [2, 2, 2, 2, 2, 2]"

rosparam set /FCTempStart true

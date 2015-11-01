#!/bin/bash

rosservice call /vehicle0/ConfigureVelocityController "ControllerName: ''
desired_mode: [2, 2, 2, 2, 2, 2]"

rosparam set /FCTempStart true

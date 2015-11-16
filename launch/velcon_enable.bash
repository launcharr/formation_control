#!/bin/bash

rosservice call /vehicle1/FormPos_enable "enable: true"
rosservice call /vehicle2/FormPos_enable "enable: true"
rosservice call /vehicle3/FormPos_enable "enable: true"

rostopic pub /FCEnable std_msgs/Bool true --once


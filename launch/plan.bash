#!/bin/bash

function pause() {
	read -n 1 -p "$*"
}

function pub_pos() {
rostopic pub /FormPosRef auv_msgs/NavSts "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
global_position: {latitude: 0.0, longitude: 0.0, altitude: 0.0}
origin: {latitude: 0.0, longitude: 0.0, altitude: 0.0}
position: {north: $1, east: $2, depth: 0.0}
altitude: 0.0
body_velocity: {x: 0.0, y: 0.0, z: 0.0}
gbody_velocity: {x: 0.0, y: 0.0, z: 0.0}
orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
orientation_rate: {roll: 0.0, pitch: 0.0, yaw: 0.0}
position_variance: {north: 0.0, east: 0.0, depth: 0.0}
orientation_variance: {roll: 0.0, pitch: 0.0, yaw: 0.0}
status: 0" --once

	pause enter
}

function pub_form() {
rostopic pub /FormChange formation_control/Formation "FormX: [0,3,6,-3,0,3,-6,-3,0]
FormY: [0,3,6,-3,0,3,-6,-3,0]
FormYaw: $1
enableParam:
- true
- true" --once

	pause enter
}

rostopic pub /FCEnable std_msgs/Bool true --once
pause enter

pub_pos 5 5

rostopic pub /AdaptFCEnable std_msgs/Bool true --once






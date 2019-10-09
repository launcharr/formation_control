#!/bin/bash

source exp_3veh.sh

function pause() {
	read -n 1 -p "$*"
}

function change_pos() {
    echo "Changing formation position to [$1,$2]";
    rostopic pub /FormPosRef auv_msgs/NavigationStatus "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
global_position: {latitude: 0.0, longitude: 0.0, altitude: 0.0}
origin: {latitude: 0.0, longitude: 0.0, altitude: 0.0}
position: {north: $1, east: $2, depth: 0.0}
altitude: 0.0
body_velocity: {x: 0.0, y: 0.0, z: 0.0}
seafloor_velocity: {x: 0.0, y: 0.0, z: 0.0}
orientation: {x: 0.0, y: 0.0, z: 0.0}
orientation_rate: {x: 0.0, y: 0.0, z: 0.0}
position_variance: {north: 0.0, east: 0.0, depth: 0.0}
orientation_variance: {x: 0.0, y: 0.0, z: 0.0}
status: 0"  --once;
}

function change_form() {
    echo "Changing formation and rotating to $5";
	rostopic pub /FormChange formation_control/Formation "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
shape:
  enable: $1
  x: $2
  y: $3
centre:
  enable: false
  x:
  - 0
  y:
  - 0
rotation:
  enable: $4
  angle: $5" --once

}

angles=(180)
#angles=(45 90 180)
#angles=(360)

#gd: n 15, e -10
#dd: n -2, e -20
#dl: n 18, e -55
#gl: n 35, e -48
positions_north=(3 9 14 7)
positions_east=(-25 -34 -27 -22)
pos_len=${#positions_north[@]}
pos_len=0

# amount of sleep for pos and form
sleep_pos=40
sleep_form45=15
sleep_form90=30
sleep_form180=40
num=0

# put initial formation shape
eval tmpx="\$${tx[0]}"
eval tmpy="\$${ty[0]}"
change_form true $tmpx $tmpy false 0.0
# start formation control
rostopic pub /FCEnable std_msgs/Bool true --once

# turn on positioning
#change_pos 7 -22
change_pos 0 0

# wati for user input when gathering finished
pause enter

# start sequence

# go through all combinations of formation 3
tx_len=${#tx[@]}
for (( k=0; k<${tx_len}; k++ ));
do
	eval tmpx="\$${tx[$k]}"
	eval tmpy="\$${ty[$k]}"
	change_form true $tmpx $tmpy false 0.0
	sleep 30
	
	for i in ${angles[@]}
	do
		for (( j=1; j<=$((360/$i)); j++ ));
		do
			change_form true $tmpx $tmpy true "$(($i * $j))"
			eval sleep_form="\$sleep_form$i"
			sleep $sleep_form
			#pause enter
			((num++))
		done
	done
	
	# change formation position
	for (( i=0; i<${pos_len}; i++ ));
	do
		change_pos ${positions_north[$i]} ${positions_east[$i]}
		sleep $sleep_pos
		#pause enter
		((num++))
	done
done

rostopic pub /FCEnable std_msgs/Bool true --once
echo "Num: $num"
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

function print_help() {

    echo "Commands:";
    echo "h - help";
    echo "form_chng - Change formation";
    echo "form_start - Start formation control";
    echo "form_stop - Stop formation control";
    echo "pos_chng - Change formation position";
    echo "exit - Exit script";
}

function check_id() {
	if [ "$1" -ge 1 ] && [ "$1" -le $2 ];
	then
		return 1
	else
		return 0
	fi
}

#gd: n 15, e -10
#dd: n -2, e -20
#dl: n 18, e -55
#gl: n 35, e -48
positions_north=(3 9 14 7)
positions_east=(-25 -34 -27 -22)
pos_len=${#positions_north[@]}

# formations
form_len=${#tx[@]}

num=0

print_help

while [ true ];
do 
	read -p "Enter command: " cmd
    case $cmd in
        h) 
            print_help
            ;;
        form_chng)
        	read -p "Choose formation [1-$form_len]: " form_id
        	check_id $form_id $form_len
			if [ $? = 0 ];
			then
				echo "Wrong formation id!"
				continue		
			fi
			eval tmpx="\$${tx[$((form_id-1))]}"
			eval tmpy="\$${ty[$((form_id-1))]}"
        	change_form true $tmpx $tmpy false 0.0
        	;;
		form_start)
			eval tmpx="\$${tx[0]}"
			eval tmpy="\$${ty[0]}"
			change_form true $tmpx $tmpy false 0.0
			# start formation control
			rostopic pub /FCEnable std_msgs/Bool true --once
			;;
		form_stop)
			# stop formation control
			rostopic pub /FCEnable std_msgs/Bool false --once
			;;
		pos_chng)
			read -p "Choose pos [1-$pos_len]: " pos_id
        	check_id $pos_id $pos_len
			if [ $? = 0 ];
			then
				echo "Wrong position id!"
				continue		
			fi
			change_pos ${positions_north[$((pos_id-1))]} ${positions_east[$((pos_id-1))]}
			;;
        exit) 
            echo "Exiting.";
            break;
            ;;
        *)
            echo "Wrong command. Type h for help"
            ;;
    esac
done
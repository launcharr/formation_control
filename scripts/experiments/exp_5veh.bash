#!/bin/bash

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

# Pravilni peterokut stranice 3:
tx1="[0.0,0.0,2.8531695488854605,4.61652530576288,2.8531695488854605,0.0,0.0,2.8531695488854605,4.61652530576288,2.8531695488854605,-2.8531695488854605,-2.8531695488854605,0.0,1.7633557568774192,0.0,-4.61652530576288,-4.61652530576288,-1.7633557568774192,0.0,-1.7633557568774192,-2.8531695488854605,-2.8531695488854605,0.0,1.7633557568774192,0.0]"
ty1="[0.0,3.0,3.9270509831248415,1.5,-0.9270509831248424,-3.0,0.0,0.9270509831248424,-1.5,-3.9270509831248415,-3.9270509831248415,-0.9270509831248424,0.0,-2.427050983124842,-4.854101966249684,-1.5,1.5,2.427050983124842,0.0,-2.427050983124842,0.9270509831248424,3.9270509831248415,4.854101966249684,2.427050983124842,0.0]"
# Tlocrt pravilne piramide duljina 5
tx2="[0.0,-2.5,2.5,2.5,-2.5,2.5,0.0,5.0,5.0,0.0,-2.5,-5.0,0.0,0.0,-5.0,-2.5,-5.0,0.0,0.0,-5.0,2.5,0.0,5.0,5.0,0.0]"
ty2="[0.0,2.5,2.5,-2.5,-2.5,-2.5,0.0,0.0,-5.0,-5.0,-2.5,0.0,0.0,-5.0,-5.0,2.5,5.0,5.0,0.0,0.0,2.5,5.0,5.0,0.0,0.0]"
# Peterokut L udaljenost 2.5m
tx3="[0.0,0.0,0.0,2.5,5.0,0.0,0.0,0.0,2.5,5.0,0.0,0.0,0.0,2.5,5.0,-2.5,-2.5,-2.5,0.0,2.5,-5.0,-5.0,-5.0,-2.5,0.0]"
ty3="[0.0,2.5,5.0,0.0,0.0,-2.5,0.0,2.5,-2.5,-2.5,-5.0,-2.5,0.0,-5.0,-5.0,0.0,2.5,5.0,0.0,0.0,0.0,2.5,5.0,0.0,0.0]"
# Linija udaljenosti 2.5
tx4="[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]"
ty4="[0.0,2.5,5.0,7.5,10.0,-2.5,0.0,2.5,5.0,7.5,-5.0,-2.5,0.0,2.5,5.0,-7.5,-5.0,-2.5,0.0,2.5,-10.0,-7.5,-5.0,-2.5,0.0]"
# Linija udaljenosti 3
tx5="[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]"
ty5="[0.0,3.0,6.0,9.0,12.0,-3.0,0.0,3.0,6.0,9.0,-6.0,-3.0,0.0,3.0,6.0,-9.0,-6.0,-3.0,0.0,3.0,-12.0,-9.0,-6.0,-3.0,0.0]"



# Choose formations
tx=(tx1 tx2 tx4)
ty=(ty1 ty2 ty4)

angles=(45 90 180)
#angles=(360)

#gd: n 15, e -10
#dd: n -2, e -20
#dl: n 18, e -55
#gl: n 35, e -48
positions_north=(6 12 17 10)
positions_east=(-23 -32 -28 -20)
pos_len=${#positions_north[@]}

# amount of sleep for pos and form
sleep_pos=45
sleep_form=30
num=0


# put initial formation shape
change_form true $tx1 $ty1 false 0.0
# start formation control
rostopic pub /FCEnable std_msgs/Bool true --once

# turn on positioning
change_pos 10 -20

# wati for user input when gathering finished
pause enter

# start sequence

# go through all combinations of formation 3
tx_len=${#tx[@]}
for (( k=0; k<${tx_len}; k++ ));
do
	for i in angles
	do
		for (( j=0; j<=$((360/$i)); j++ ));
		do
			eval tmpx="\$${tx[$k]}"
			eval tmpy="\$${ty[$k]}"
			change_form true $tmpx $tmpy true "$(($i * $j))" 
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

echo "Num: $num"

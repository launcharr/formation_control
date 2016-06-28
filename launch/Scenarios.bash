#!/bin/bash

function pause() {
	read -n 1 -p "$*"
}

function change_pos() {
    echo "Changing formation position.";
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
status: 0" --once;

}

function change_form() {
    echo "Changing formation.";
    rostopic pub /FormChange formation_control/Formation "FormX: $1
FormY: $2
FormYaw: $3
enableParam:
- $4
- $5" --once;
}

function FC_enable() {
    if [ $1 == true ]; then
        echo "Enabling formation control.";
        rostopic pub /FCEnable std_msgs/Bool true --once;
    elif [ $1 == false ]; then
        echo "Disabling formation control.";
        rostopic pub /FCEnable std_msgs/Bool false --once;
    else echo "FC_enable requires true or false parameter."
    fi
}

function print_help() {

    echo "Commands:";
    echo "h - help";
    echo "fc_en - Enable formation control";
    echo "fc_dis - Disable formation control";
    echo "chg_pos - Change formation position";
    echo "chg_form - Change formation";
    echo "s1 - Start scenario 1";
    echo "s2 - Start scenario 2";
    echo "s3 - Start scenario 3";
    echo "s4 - Start scenario 4";
}
i=0;
while [ true ];
    do read -p "Enter command: " cmd
    i=i+1;
    case $cmd in
        h) 
            print_help
            ;;
        fc_en)
            FC_enable true;
            ;;
        fc_dis)
            FC_enable false;
            ;;
        chg_pos)
            read -p "Enter x position:" x;
            read -p "Enter y position:" y;
            change_pos $x $y;
            ;;
        chg_form)
            read -p "Change formation: (true/false)" par1_en;
            read -p "Change formation yaw: (true/false)" par2_en;
            if [ "$par1_en" = "true" ]; then
                read -p "Enter FormX: ([dx11, dx12, dx13, dx21, dx22, dx23, dx31, dx32, dx33])" formX;
                read -p "Enter FormY: ([dy11, dy12, dy13, dy21, dy22, dy23, dy31, dy32, dy33])" formY;
            else
                formX=[0,3,6,-3,0,3,-6,-3,0];
                formY=[0,3,6,-3,0,3,-6,-3,0];
            fi
            if [ "$par2_en" = "true" ]; then
                read -p "Enter formation yaw:" yaw;
            else
                yaw=0;
            fi
            change_form $formX $formY $yaw $par1_en $par2_en
            ;;
                
        s1)
            # scenarij krairanja formacije
            echo "Scenario 1 starting.";
            # pocni snimat bag
            rosbag record /vehicle1/stateHat /vehicle2/stateHat /vehicle3/stateHat /FCEnable /FormPosRef /FormChange -o Scenario1 & 
            pid=$(echo $!)
            # upali regulator
            FC_enable true;
            change_form [0,3,1.5,-3,0,-1.5,-1.5,1.5,0] [0,0,3,0,0,3,-3,-3,0] 0 true false
            pause enter
            # nakon kreiranja gasi rosbag i regulator
            pkill -INT -P $pid
            FC_enable false;
            ;;
        s2)
            # scenarij promjene formacije
            echo "Scenario 2 starting.";
            # upali regulator
            FC_enable true;
            change_form [0,3,1.5,-3,0,-1.5,-1.5,1.5,0] [0,0,3,0,0,3,-3,-3,0] 0 true false
            pause "Pocni snimat?";
            # pocni snimat bag
            rosbag record /vehicle1/stateHat /vehicle2/stateHat /vehicle3/stateHat /FCEnable /FormPosRef /FormChange -o Scenario2 &
            pid=$(echo $!)
            # promijeni formaciju
            change_form [0,-4,-2,4,0,2,2,-2,0] [0,0,-3.46,0,0,-3.46,3.46,3.46,0] 0.0 true false
            pause "Formacija promjenjena?"
            # nakon kreiranja gasi rosbag i regulator
            pkill -INT -P $pid
            FC_enable false;
            ;;
        s3)
            # scenarij rotacije i translacije formacije
            echo "Scenario 3 starting.";
            FC_enable true;
            change_form [0,3,1.5,-3,0,-1.5,-1.5,1.5,0] [0,0,3,0,0,3,-3,-3,0] 0 true false
            change_pos 0 0;
            pause "Pocni snimat?";
            # pocni snimat bag
            rosbag record /vehicle1/stateHat /vehicle2/stateHat /vehicle3/stateHat /FCEnable /FormPosRef /FormChange -o Scenario3 &
            pid=$(echo $!)
            change_form [0,2,4,-2,0,2,-4,-2,0] [0,2,4,-2,0,2,-4,-2,0] 80.0 false true
            pause "Rotacija gotova? Next: Translacija 1!";
            change_pos 6 6;
            pause "Translacija 1 gotova? Next: Translacija 2! End!";
            change_pos 4 -8;
            pause "Translacija 2 gotova? End!";
            # nakon kreiranja gasi rosbag i regulator
            pkill -INT -P $pid
            FC_enable false;
            ;;
        s4)
            # scenarij dinamickog pozicioniranja?
            echo "Scenario 4 starting.";
            # pocni snimat bag
            rosbag record /vehicle1/stateHat /vehicle2/stateHat /vehicle3/stateHat /FCEnable /FormPosRef /FormChange -o Scenario4 &
            pid=$(echo $!)
            # upali regulator
            FC_enable true;
            
            
            pause enter
            # nakon scenarija gasi rosbag i regulator
            pkill -INT -P $pid
            FC_enable false;
            ;;
        rst)
            echo "Reseting formation";
            FC_enable true;
            change_form [0,-10.0,0,10.0,0,10.0,0,-10.0,0] [0,0,-10.0,0,0,-10.0,10.0,10.0,0] 0.0 true false
            change_pos 1.67 1.67;
            pause "Formation reset?"
            FC_enable false;
            ;;
        ri)
            echo "Reinitializing";
            rosparam load $(rospack find formation_control)/config/config.yaml /vehicle1;
            rosparam load $(rospack find formation_control)/config/config.yaml /vehicle2;
            rosparam load $(rospack find formation_control)/config/config.yaml /vehicle3;
            rostopic pub /FCReinit std_msgs/Bool true --once;
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
    







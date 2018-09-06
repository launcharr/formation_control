source env_formation_veh_local.sh

BYOBU_SESSION_NAME=formation

# get length of an array
len=${#VEH_NAME[@]}

#roslaunch -p $LOCAL_PORT formation_control formation_topside.launch
byobu new-session -d -s "$BYOBU_SESSION_NAME" "roslaunch -p $LOCAL_PORT formation_control formation_topside.launch && basl -l"

# start every formation control node
for (( i=0; i<${len}; i++ ));
do
    comm="export ROS_MASTER_URI=http://${VEH_IP[$i]}:${VEH_PORT[$i]} && roslaunch formation_control formation_pladypos.launch veh_name:=${VEH_NAME[$i]} veh_id:=$i && bash -l"
    byobu new-window -n "${VEH_NAME[$i]} fc" -t "$BYOBU_SESSION_NAME" "$comm"
done

# export ros master
for (( i=0; i<${len}; i++ ));
do
    #byobu new-window -n "${VEH_NAME[$i]} echo" -t "formation" "echo $i && bash -l"
    comm="export ROS_MASTER_URI=http://${VEH_IP[$i]}:${VEH_PORT[$i]} && bash -l"
    byobu new-window -n "${VEH_NAME[$i]} ${VEH_PORT[$i]}" -t "formation" "$comm"
done

byobu attach -t "$BYOBU_SESSION_NAME"
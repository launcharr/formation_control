# load formation environment variables
source env_formation_veh.sh

BYOBU_SESSION_NAME=formation
LOCAL_IP=10.0.10.41
ROS2UDP_PORT=39876

# get length of an array
len=${#VEH_NAME[@]}

byobu new-session -d -s "$BYOBU_SESSION_NAME" "roslaunch -p $LOCAL_PORT formation_control formation_topside.launch local_simulation:=false veh_num:=$len local_address:=$LOCAL_IP local_port:=$ROS2UDP_PORT && bash -l"

# start every formation control node
for (( i=0; i<${len}; i++ ));
do
    comm="ssh -t stdops@${VEH_IP[$i]} 'source ~/subcultron_ws/devel/setup.bash && roslaunch formation_control formation_${VEH_NAME[$i]}.launch veh_num:=$len veh_name:=${VEH_NAME[$i]} veh_id:=$i local_address:=${VEH_IP[$i]} local_port:=$ROS2UDP_PORT && bash -l'"
    byobu new-window -n "${VEH_NAME[$i]} fc" -t "$BYOBU_SESSION_NAME" "$comm"
done

# export ros master
for (( i=0; i<${len}; i++ ));
do
    comm="source ~/subcultron_ws/devel/setup.bash && export ROS_MASTER_URI=http://${VEH_IP[$i]}:${VEH_PORT[$i]} && bash -l"
    byobu new-window -n "${VEH_NAME[$i]} ${VEH_PORT[$i]}" -t "formation" "$comm"
done

byobu attach -t "$BYOBU_SESSION_NAME"

# load formation environment variables
source env_formation_veh.sh

BYOBU_SESSION_NAME=formation
LOCAL_IP=10.0.10.41
ROS2UDP_PORT=39876

VEH_ID=(1 2 3 4 5)
len=${#VEH_ID[@]}

for (( i=0; i<${len}; i++ ));
do
	if [ ${VEH_ID[$i]} -le ${#VEH_NAMES[@]} ]; then
		VEH_NAME[$i]=${VEH_NAMES[$((${VEH_ID[$i]}-1))]}
		VEH_PORT[$i]=${VEH_PORTS[$((${VEH_ID[$i]}-1))]}
		VEH_IP[$i]=${VEH_IPS[$((${VEH_ID[$i]}-1))]}
	else
		echo "ID out of range!"
		exit
	fi
done

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

byobu new-window -n "command" -t "$BYOBU_SESSION_NAME" "/experiments/exp_4veh.bash && bash -l"

byobu attach -t "$BYOBU_SESSION_NAME"

source env_formation_veh_local.sh

BYOBU_SESSION_NAME=simulation

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

byobu new-session -d -s "$BYOBU_SESSION_NAME" "echo 1 && bash -l"

# start every formation control node
for (( i=0; i<${len}; i++ ));
do
    comm="source ~/opt/labust/pladypos_${VEH_NAME[$i]}_env.sh && roslaunch -p ${VEH_PORT[$i]} pladypos mws.launch && bash -l"
    byobu new-window -n "${VEH_NAME[$i]} sim" -t "$BYOBU_SESSION_NAME" "$comm"
done

byobu attach -t "$BYOBU_SESSION_NAME"

source env_formation_veh_local.sh

BYOBU_SESSION_NAME=simulation

# get length of an array
len=${#VEH_NAME[@]}

#roslaunch -p $LOCAL_PORT formation_control formation_topside.launch
byobu new-session -d -s "$BYOBU_SESSION_NAME" "echo 1 && bash -l"

# start every formation control node
for (( i=0; i<${len}; i++ ));
do
    comm="source ~/opt/labust/pladypos_${VEH_NAME[$i]}_env.sh && roslaunch -p ${VEH_PORT[$i]} pladypos mws.launch && bash -l"
    byobu new-window -n "${VEH_NAME[$i]} sim" -t "$BYOBU_SESSION_NAME" "$comm"
done

byobu attach -t "$BYOBU_SESSION_NAME"
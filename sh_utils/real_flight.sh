gnome-terminal --window -e 'bash -c "sleep 3; roslaunch ctrl_node run_ctrl.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch planning route_follow.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0; "' \
--tab -e 'bash -c "sleep 6; rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0; "' \
--tab -e 'bash -c "sleep 6; rosrun mavros mavcmd long 511 32 5000 0 0 0 0 0; "' \
--tab -e 'bash -c "sleep 6; rosrun mavros mavcmd long 511 331 5000 0 0 0 0 0; exec bash"' \
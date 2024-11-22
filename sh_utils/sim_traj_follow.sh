gnome-terminal --window -e 'bash -c "roslaunch ctrl_node base_sim_single_vehicle_ground_truth.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch ctrl_node run_ctrl.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch planning route_follow.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0; "' \
--tab -e 'bash -c "sleep 6; rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0; "' \
--tab -e 'bash -c "sleep 6; rosrun mavros mavcmd long 511 32 5000 0 0 0 0 0; "' \
--tab -e 'bash -c "sleep 6; rosrun mavros mavcmd long 511 331 5000 0 0 0 0 0; exec bash"' \

# gnome-terminal --window -e 'bash -c "roslaunch simulation_utils smartcar_display.launch; exec bash"' \
# --tab -e 'bash -c "sleep 2; roslaunch simulation_utils control.launch; exec bash"' \
# --tab -e 'bash -c "sleep 4; rosrun simulation_utils cmdvel2gazebo_keyboard; exec bash"' \

# --tab -e 'bash -c "sleep 16; roslaunch realflight_utils traj_analyse.launch; exec bash"' \

#  --tab -e 'bash -c "sleep 3; rosrun rosserial_server socket_node /mavros/vision_pose/pose; exec bash"' \
# --tab -e 'bash -c "sleep 3; roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0"; exec bash"' \

# --tab -e 'bash -c "sleep 5; rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0; "' \
# --tab -e 'bash -c "sleep 6; rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0; "' \
# --tab -e 'bash -c "sleep 6; rosrun mavros mavcmd long 511 32 5000 0 0 0 0 0; "' \
# --tab -e 'bash -c "sleep 7; rosrun mavros mavcmd long 511 331 5000 0 0 0 0 0; exec bash"' \

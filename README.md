# Introduction
## Simulation Step
Run the following script in {PROJECT_FILE}/sh_utils to start simulation:
```
./sim_traj_follow.sh
```
If you are using Software-in-the-Loop environment, run the following cmd to OFFBOARD the vehicle:
```
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'"
```
run takeoff.sh to takeoff the vehicle while vehicle change to OFFBOARD mode successfully.

#!/bin/bash

# Expecting these two rosparam to be already set
NUM_DRONES=$(rosparam get /number_of_drones)
DRONE_NAME=$(rosparam get /drone_name)

echo "Creating $NUM_DRONES nbv selector"

# Loop through the number of drones and launch each
for ((i=0; i<NUM_DRONES; i++)); do
    namespace="${DRONE_NAME}_${i}"
    roslaunch mav_local_planner mav_local_planner.launch namespace:=$namespace &
done

# Wait for all launched processes to complete
wait
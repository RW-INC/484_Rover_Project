#!/bin/bash

#!/usr/bin/env bash
killall -9 xterm
source install/setup.bash
colcon build
source install/setup.bash

countdown() {
    for i in $(seq "$1" -1 1); do
        printf "\r\033[0K%s in %ds" "$2" "$i"
        sleep 1
    done
    printf "\n"
}

xterm -hold -T launch -e bash -c "PYTHONUNBUFFERED=1 stdbuf -oL -eL ros2 launch build_sparxe sparxe.launch.py" &
countdown 10 "launching nodes"

xterm -hold -T auto_node -e bash -c "PYTHONUNBUFFERED=1 stdbuf -oL -eL ros2 run autonomy auto_node" &
xterm -hold -T smc_node  -e bash -c "PYTHONUNBUFFERED=1 stdbuf -oL -eL ros2 run motor_commander smc_node" &
xterm -hold -T auto_pub_node -e bash -c "PYTHONUNBUFFERED=1 stdbuf -oL -eL ros2 run nav_filters auto_pub_node" &

wait

#!/bin/bash

trap : SIGTERM SIGINT

# campus forest garage indoor tunnel
ENV_TYPE="garage"
SCRIPT_PATH="$(readlink -f "$0")"
WORKSPACE_DIR="$(dirname "$(dirname "$(dirname "$(dirname "$(dirname "$SCRIPT_PATH")")")")")"
# echo "Script directory is: ${WORKSPACE_DIR}"

gnome-terminal -t "env" --tab -- bash -c "source ${WORKSPACE_DIR}/autonomous_exploration_development_environment/devel/setup.bash && \
                                  roslaunch vehicle_simulator system_${ENV_TYPE}.launch ; exec bash"

sleep 1s

gnome-terminal -t "tare" --tab -- bash -c "source ${WORKSPACE_DIR}/tare_planner/devel/setup.bash && \
                                   roslaunch tare_planner explore_${ENV_TYPE}.launch ; exec bash"
#!/usr/bin/env bash
set -euo pipefail

WS="${HOME}/ros2_tb4_ws"
MAP_PATH="${WS}/src/turtlebot4/turtlebot4_navigation/maps/maze.yaml"
LOG_DIR="${WS}/.ros_logs"

if [[ ! -f "${MAP_PATH}" ]]; then
  echo "[ERROR] map file not found: ${MAP_PATH}"
  exit 1
fi

cleanup_old_processes() {
  echo "[INFO] stopping old simulation/localization processes..."

  # Stop launch parents first
  pkill -f "ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py" || true
  pkill -f "ros2 launch turtlebot4_navigation localization.launch.py" || true
  pkill -f "ros2 launch turtlebot4_navigation nav2.launch.py" || true

  # Stop common children to avoid duplicate node names
  pkill -f "/nav2_map_server/map_server" || true
  pkill -f "/nav2_amcl/amcl" || true
  pkill -f "lifecycle_manager_localization" || true
  pkill -f "lifecycle_manager_navigation" || true
  pkill -f "/bt_navigator" || true
  pkill -f "/controller_server" || true
  pkill -f "/planner_server" || true
  pkill -f "ign gazebo" || true
  pkill -f "gz sim" || true
  pkill -f "rviz2" || true

  sleep 1
}

cleanup_on_exit() {
  echo
  echo "[INFO] caught exit signal, cleaning up..."
  pkill -f "ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py" || true
  pkill -f "ros2 launch turtlebot4_navigation localization.launch.py" || true
}

trap cleanup_on_exit INT TERM

# ROS setup scripts may read unset vars; disable nounset temporarily.
set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

mkdir -p "${LOG_DIR}"
export ROS_LOG_DIR="${LOG_DIR}"

cleanup_old_processes

echo "[INFO] launching maze simulation..."
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=maze rviz:=true &
SIM_PID=$!

sleep 5

echo "[INFO] launching localization with maze map..."
ros2 launch turtlebot4_navigation localization.launch.py use_sim_time:=true map:="${MAP_PATH}" &
LOC_PID=$!

echo "[INFO] started."
echo "  simulation pid:   ${SIM_PID}"
echo "  localization pid: ${LOC_PID}"
echo
echo "Next step in RViz: click '2D Pose Estimate' once to set initial pose."

wait

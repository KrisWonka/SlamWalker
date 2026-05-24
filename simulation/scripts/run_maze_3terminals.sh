#!/usr/bin/env zsh
set -euo pipefail

# Workspace and default map settings.
# - Change MAP_PATH to switch localization map yaml.
# - The simulation world is configured in CMD_SIM below (world:=maze).
WS="${HOME}/ros2_tb4_ws"
MAP_PATH="${WS}/src/turtlebot4/turtlebot4_navigation/maps/maze.yaml"
LOG_DIR="${WS}/.ros_logs"
WORLD_NAME="maze"

if [[ ! -f "${MAP_PATH}" ]]; then
  echo "[ERROR] map file not found: ${MAP_PATH}"
  exit 1
fi

launch_in_terminal() {
  local title="$1"
  local cmd="$2"

  # Open each ROS launch in its own terminal tab/window.
  # Use the first available terminal emulator.
  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="${title}" -- zsh -lc "${cmd}"
  elif command -v xfce4-terminal >/dev/null 2>&1; then
    xfce4-terminal --title="${title}" -e "zsh -lc '${cmd}'"
  elif command -v konsole >/dev/null 2>&1; then
    konsole --new-tab -p tabtitle="${title}" -e zsh -lc "${cmd}"
  elif command -v xterm >/dev/null 2>&1; then
    xterm -T "${title}" -e zsh -lc "${cmd}" &
  else
    echo "[ERROR] no supported terminal emulator found."
    echo "Install one of: gnome-terminal / xfce4-terminal / konsole / xterm"
    exit 1
  fi
}

cleanup_old_processes() {
  echo "[INFO] stopping old simulation/localization/navigation processes..."

  # Stop parent launch commands first.
  pkill -f "ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py" || true
  pkill -f "ros2 launch turtlebot4_navigation localization.launch.py" || true
  pkill -f "ros2 launch turtlebot4_navigation nav2.launch.py" || true

  # Stop common child nodes to avoid duplicate node-name conflicts.
  pkill -f "/nav2_map_server/map_server" || true
  pkill -f "/nav2_amcl/amcl" || true
  pkill -f "lifecycle_manager_localization" || true
  pkill -f "lifecycle_manager_navigation" || true
  pkill -f "/bt_navigator" || true
  pkill -f "/controller_server" || true
  pkill -f "/planner_server" || true
  pkill -f "loop_chain_stream.py" || true
  pkill -f "nav_loop_monitor.py" || true

  pkill -f "ign gazebo" || true
  pkill -f "gz sim" || true
  pkill -f "rviz2" || true

  # Small delay to let process teardown settle.
  sleep 1
}

cleanup_on_exit() {
  echo
  echo "[INFO] cleaning up all launched processes..."
  cleanup_old_processes
}

ensure_localization_active() {
  echo "[INFO] checking localization lifecycle state..."
  eval "${COMMON_ENV} && ros2 lifecycle get /map_server >/tmp/tb4_map_state.txt 2>/dev/null || true"
  eval "${COMMON_ENV} && ros2 lifecycle get /amcl >/tmp/tb4_amcl_state.txt 2>/dev/null || true"

  if rg -q "inactive" "/tmp/tb4_map_state.txt" || rg -q "inactive" "/tmp/tb4_amcl_state.txt"; then
    echo "[WARN] localization nodes inactive, activating map_server/amcl..."
    eval "${COMMON_ENV} && ros2 lifecycle set /map_server activate || true"
    eval "${COMMON_ENV} && ros2 lifecycle set /amcl activate || true"
  fi
}

wait_for_map_tf() {
  local max_wait_s="${1:-20}"
  local start_ts now elapsed
  start_ts="$(date +%s)"
  echo "[INFO] waiting for TF map -> base_link (timeout: ${max_wait_s}s)..."

  while true; do
    if eval "${COMMON_ENV} && timeout 1 ros2 run tf2_ros tf2_echo map base_link >/tmp/tb4_tf_check.txt 2>&1"; then
      echo "[INFO] TF map -> base_link is available."
      return 0
    fi

    now="$(date +%s)"
    elapsed=$((now - start_ts))
    if (( elapsed >= max_wait_s )); then
      echo "[WARN] TF map -> base_link not ready after ${max_wait_s}s."
      echo "[WARN] You can still continue, but Nav2 may report map-frame timeout until initial pose is set."
      return 1
    fi
    sleep 1
  done
}

trap cleanup_on_exit INT TERM

# Put ROS launch logs under workspace for easier debugging.
mkdir -p "${LOG_DIR}"

# Shared runtime environment for all launched terminals.
# setup.zsh may read unset vars; temporarily disable nounset around source.
COMMON_ENV="set +u && source /opt/ros/humble/setup.zsh && source ${WS}/install/setup.zsh && set -u && export ROS_LOG_DIR=${LOG_DIR}"
# Monitor must use ROS-compatible Python (Humble -> Python 3.10), not conda python.
if [[ -x /usr/bin/python3.10 ]]; then
  MONITOR_PY="/usr/bin/python3.10"
elif command -v python3.10 >/dev/null 2>&1; then
  MONITOR_PY="$(command -v python3.10)"
else
  MONITOR_PY="/usr/bin/python3"
fi
# Terminal 1: simulator + RViz (change WORLD_NAME above if needed).
CMD_SIM="${COMMON_ENV} && ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=${WORLD_NAME} rviz:=true"
# Terminal 2: localization (AMCL + map server).
CMD_LOC="${COMMON_ENV} && ros2 launch turtlebot4_navigation localization.launch.py use_sim_time:=true map:=${MAP_PATH}"
# Terminal 3: Nav2 stack.
CMD_NAV2="${COMMON_ENV} && ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true"
MONITOR_CMD="${COMMON_ENV} && unset PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL _CE_CONDA _CE_M && echo \"[INFO] monitor python: ${MONITOR_PY}\" && ${MONITOR_PY} ${WS}/scripts/nav_loop_monitor.py ${WORLD_NAME} ${MAP_PATH}"
LOOP_CHAIN_CMD="${COMMON_ENV} && unset PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL _CE_CONDA _CE_M && ${MONITOR_PY} ${WS}/scripts/loop_chain_stream.py"

cleanup_old_processes

echo "[INFO] opening terminal 1: simulation"
launch_in_terminal "TB4 Maze Sim" "${CMD_SIM}"
sleep 6

echo "[INFO] opening terminal 2: localization"
launch_in_terminal "TB4 Maze Localization" "${CMD_LOC}"
sleep 4
ensure_localization_active
wait_for_map_tf 20 || true

echo "[INFO] opening terminal 3: nav2"
launch_in_terminal "TB4 Maze Nav2" "${CMD_NAV2}"
sleep 2

echo "[INFO] opening terminal 4: loop chain stream"
launch_in_terminal "TB4 Loop Chain Stream" "${LOOP_CHAIN_CMD}"

echo "[INFO] all three terminals launched."
echo "In RViz: click '2D Pose Estimate' once, then send 'Nav2 Goal'."
echo "Main terminal now runs a subscriber-based real-time monitor."
echo "Press Esc in this terminal to stop all processes and exit."

# Run precise state monitor; it exits on Esc.
eval "${MONITOR_CMD}"
cleanup_on_exit
echo "[INFO] stopped."

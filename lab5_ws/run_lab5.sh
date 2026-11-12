#!/usr/bin/env bash
# Run from any directory; the workspace is located relative to this script.
# Defaults match the repository's do_config.sh, without replacing caller values.

if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  printf '%s\n' 'Run this script as ./run_lab5.sh; do not source it.' >&2
  return 2
fi

set -eo pipefail
# ROS setup scripts may reference unset variables, so do not enable nounset.

LAB5_SCRIPT_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
LAB5_SCRIPT_PACKAGE="$LAB5_SCRIPT_WS/turtlebot3_recovery_lab"

usage() {
  cat <<'USAGE'
Usage: ./run_lab5.sh <build|bringup|waypoints> [arguments...]

  build       Build the Lab 5 package; extra arguments go to colcon build.
  bringup     Start Nav2, recovery services and RViz; does not send goals.
  waypoints   Start the configured route after localization is ready.
  help        Show this help without requiring ROS.

Examples (run on the Ubuntu / ROS 2 Humble workstation):
  ./run_lab5.sh build
  ./run_lab5.sh bringup
  ./run_lab5.sh bringup map:=/absolute/path/to/maze.yaml rviz:=False
  ./run_lab5.sh waypoints goal_timeout_sec:=300.0
  ./run_lab5.sh waypoints "mission_file:=/path with spaces/mission.yaml"

Defaults, relative to this script's workspace:
  map:          maps/maze.yaml
  mission_file: turtlebot3_recovery_lab/config/mission.yaml
  params_file:  turtlebot3_recovery_lab/config/nav2_params.yaml
  use_sim_time: False (override in BOTH terminals for simulation)

Environment defaults: TURTLEBOT3_MODEL=burger, ROS_DOMAIN_ID=66, LDS_MODEL=LDS-02.
Existing nonempty environment values are preserved.
LAB5_ROS_SETUP overrides /opt/ros/humble/setup.bash.
LAB5_TURTLEBOT3_SETUP selects an optional TurtleBot3 overlay setup.bash;
otherwise ~/turtlebot3_ws/install/setup.bash is loaded when present.

The robot drivers must already be running. Fill in the mission coordinates and
save your map first. Use separate terminals for bringup and waypoints; initialize
AMCL in RViz before starting waypoints. This script does not install dependencies.
USAGE
}

fail() {
  printf 'Lab 5: %s\n' "$*" >&2
  exit 1
}

command_name="${1:-help}"
if [[ $# -gt 0 ]]; then
  shift
fi
case "$command_name" in
  help|-h|--help)
    usage
    exit 0
    ;;
  build|bringup|waypoints)
    ;;
  *)
    printf 'Unknown command: %s\n' "$command_name" >&2
    usage >&2
    exit 2
    ;;
esac
if [[ "${1:-}" == '-h' || "${1:-}" == '--help' ]]; then
  usage
  exit 0
fi

ros_setup="${LAB5_ROS_SETUP:-/opt/ros/humble/setup.bash}"
[[ -r "$ros_setup" ]] || fail "ROS setup not found: $ros_setup. Run on the Humble workstation, or set LAB5_ROS_SETUP."
# shellcheck disable=SC1090
source "$ros_setup"

if [[ -n "${LAB5_TURTLEBOT3_SETUP:-}" ]]; then
  [[ -r "$LAB5_TURTLEBOT3_SETUP" ]] || fail "TurtleBot3 setup not found: $LAB5_TURTLEBOT3_SETUP"
  # shellcheck disable=SC1090
  source "$LAB5_TURTLEBOT3_SETUP"
elif [[ -r "$HOME/turtlebot3_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$HOME/turtlebot3_ws/install/setup.bash"
fi

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"
export LDS_MODEL="${LDS_MODEL:-LDS-02}"

if [[ "$command_name" == 'build' ]]; then
  command -v colcon >/dev/null 2>&1 || fail 'colcon is unavailable; install the ROS build tools first.'
  cd -- "$LAB5_SCRIPT_WS"
  exec colcon build --symlink-install --base-paths "$LAB5_SCRIPT_PACKAGE" \
    --packages-select turtlebot3_recovery_lab "$@"
fi

[[ -r "$LAB5_SCRIPT_WS/install/setup.bash" ]] || fail 'Workspace is not built. Run ./run_lab5.sh build first.'
# shellcheck disable=SC1091
source "$LAB5_SCRIPT_WS/install/setup.bash"
command -v ros2 >/dev/null 2>&1 || fail 'ros2 is unavailable after sourcing the workspace.'

# Add defaults only for arguments the caller did not supply. Preserve each
# argument as one array element, including paths with spaces; never use eval.
launch_args=("$@")
launch_default() {
  local key="$1" value="$2" argument
  for argument in "${launch_args[@]}"; do
    if [[ "$argument" == "$key:="* ]]; then
      return
    fi
  done
  launch_args+=("$key:=$value")
}

launch_default mission_file "$LAB5_SCRIPT_PACKAGE/config/mission.yaml"
launch_default use_sim_time False
printf 'Lab 5: %s | model=%s | ROS_DOMAIN_ID=%s | LiDAR=%s\n' \
  "$command_name" "$TURTLEBOT3_MODEL" "$ROS_DOMAIN_ID" "$LDS_MODEL"

if [[ "$command_name" == 'bringup' ]]; then
  launch_default map "$LAB5_SCRIPT_WS/maps/maze.yaml"
  launch_default params_file "$LAB5_SCRIPT_PACKAGE/config/nav2_params.yaml"
  exec ros2 launch turtlebot3_recovery_lab lab5_bringup.launch.py "${launch_args[@]}"
fi

exec ros2 launch turtlebot3_recovery_lab waypoints.launch.py "${launch_args[@]}"

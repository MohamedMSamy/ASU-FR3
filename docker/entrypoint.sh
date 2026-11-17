#!/usr/bin/env bash
# ASU-FR3 container entrypoint: source ROS and the workspace, select the DDS implementation, run the command.
#   RMW=cyclone (default) | fastdds       ROS_NETWORK_INTERFACE=lo (default, Cyclone only)
# Replaces the demo's scripts/setup_middleware.sh. Cyclone reads config/$CYCLONE_CONFIG (default cyclone_config.xml; `./fr3 up isaac`
# uses cyclone_isaac.xml) - a file of that name mounted at /ws_override/config wins over the baked default.
set -e
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source /home/ros/ros2_ws/install/setup.bash
case "${RMW:-cyclone}" in
  cyclone)
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_NETWORK_INTERFACE="${ROS_NETWORK_INTERFACE:-lo}"
    cfg="${CYCLONE_CONFIG:-cyclone_config.xml}"
    conf="${ASU_FR3_DIR:-/opt/asu_fr3}/config/$cfg"
    [ -f "${ASU_FR3_OVERRIDE:-/ws_override}/config/$cfg" ] && conf="${ASU_FR3_OVERRIDE:-/ws_override}/config/$cfg"
    [ -f "$conf" ] || { echo "asu-fr3-entrypoint: Cyclone config $cfg not found" >&2; exit 1; }
    export CYCLONEDDS_URI="file://$conf"
    ;;
  fastdds|fast)
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    ;;
  *)
    echo "unknown RMW '${RMW}' (use cyclone or fastdds)" >&2
    exit 1
    ;;
esac
# ./data is mounted from the host; if it belongs to another user (or to root, when Docker created it) make it writable
if [ -d /home/ros/data ] && [ ! -w /home/ros/data ]; then sudo -n chmod a+rwx /home/ros/data 2>/dev/null || echo "[asu_fr3] warning: /home/ros/data is not writable (logs cannot be saved)" >&2; fi
exec "$@"

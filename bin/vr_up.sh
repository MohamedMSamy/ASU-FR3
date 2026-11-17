#!/usr/bin/env bash
# Start the Quest 2 teleop INSIDE the running stack's container: ROS-TCP endpoint (headset link) + the vr_* tools.
# Ctrl-C stops all of them.  Normally started with ./fr3 vr [mode] [options]  (copy of crisp_lab/bin/vr_up.sh).
#   ./fr3 vr                          # sim, pos 0.5, rot 0.2
#   ./fr3 vr mujoco --pos 0.5 --rot 0.2
#   POS=0.4 VR_IP=10.0.0.5 ./fr3 vr
# It never switches controllers: activate cartesian_impedance_controller yourself first
# (./fr3 ctl mujoco switch_controllers --deactivate joint_trajectory_controller --activate cartesian_impedance_controller).
# Options: --pos S     position scale, hand->arm motion ratio (default 0.5, or $POS)
#          --rot S     rotation scale (default 0.2, or $ROT; 0 keeps orientation fixed)
#          --ip IP     laptop IP the headset connects to (default: first of `hostname -I`, or $VR_IP)
#          --port P    endpoint TCP port (default 10000)
# Change the scales while running (second terminal):  ./fr3 scale --pos 0.5 --rot 0.3
# VR_TELEOP=vr_teleop_clamped.py ./fr3 vr   runs the variant that clamps the target to the safety box of lab_config.py.
set -eo pipefail
MODE="${CRISP_LAB_MODE:-sim}"
POS="${POS:-0.5}"
ROT="${ROT:-0.2}"
IP="${VR_IP:-$(hostname -I | awk '{print $1}')}"
PORT=10000
while [ $# -gt 0 ]; do
  case "$1" in
    --pos|--scale) POS="${2:?--pos needs a value}"; shift 2 ;;
    --rot) ROT="${2:?--rot needs a value}"; shift 2 ;;
    --ip) IP="${2:?--ip needs a value}"; shift 2 ;;
    --port) PORT="${2:?--port needs a value}"; shift 2 ;;
    -h|--help) sed -n '2,14p' "$0"; exit 0 ;;
    *) echo "unknown option: $1 (see --help)"; exit 1 ;;
  esac
done
[ -n "$IP" ] || { echo "could not detect the laptop IP, pass --ip"; exit 1; }
# tools/ in the repo (mounted at /ws_override) win over the baked copies
tool() { for d in /ws_override/tools /opt/asu_fr3/tools; do [ -f "$d/$1" ] && { echo "$d/$1"; return; }; done; echo "no such tool: $1" >&2; exit 1; }
TELEOP="$(tool "${VR_TELEOP:-vr_teleop.py}")"

echo "[vr] reminder: switch to cartesian_impedance_controller yourself (./fr3 ctl ...) before pressing the grip"
echo "[vr] mode=$MODE  endpoint=$IP:$PORT  pos=$POS rot=$ROT"
echo "[vr] headset: Quest2ROS -> ROS 2 -> $IP : $PORT.  Hold right grip to move."
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:="$IP" -p ROS_TCP_PORT:="$PORT" &
trap 'kill $(jobs -p) 2>/dev/null; wait 2>/dev/null' EXIT
sleep 2
python3 "$(tool vr_target_viz.py)" &
if [ "${GRIPPER:-1}" != 0 ]; then python3 "$(tool vr_gripper.py)" & fi
python3 "$TELEOP" --ros-args -p pos_scale:="$POS" -p rot_scale:="$ROT" -p max_lead:=0.03 -p max_rot_lead:=0.10

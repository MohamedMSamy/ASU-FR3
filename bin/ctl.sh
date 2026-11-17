#!/usr/bin/env bash
# Container-local replacement for crisp_lab/bin/ctl.sh, used by the scripts in scripts/ when they run INSIDE the stack's
# container (ROS 2 Jazzy on both sides, so the controller list is read correctly). The first argument (sim|real) is ignored.
#   ctl.sh sim                              list the controllers
#   ctl.sh sim switch_controllers --deactivate A --activate B
#   ctl.sh sim ros2 topic hz /clock         any ros2 command, stopped after CTL_TIMEOUT (default 10) seconds
set -euo pipefail
shift || true
VERB="${1:-list_controllers}"; [ $# -gt 0 ] && shift
if [ "$VERB" = "ros2" ]; then exec timeout "${CTL_TIMEOUT:-10}" ros2 "$@"; fi
exec ros2 control "$VERB" "$@"

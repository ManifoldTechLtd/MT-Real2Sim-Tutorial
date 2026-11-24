#!/usr/bin/env bash
# Send the preset waypoint mission to the running ONS2 stack.
set -uo pipefail

RUNTIME="ros2"
SCENE="office_0923_sim"
CONTAINER="isaac_nav_ros2"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --runtime) RUNTIME="$2"; shift 2 ;;
    --scene) SCENE="$2"; shift 2 ;;
    --container) CONTAINER="$2"; shift 2 ;;
    *) echo "unknown arg: $1"; exit 1 ;;
  esac
done

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
WP_FILE="${ROOT}/integrations/ons2/waypoints/${SCENE}.yaml"
[[ -f "${WP_FILE}" ]] || { echo "missing waypoints: ${WP_FILE}"; exit 1; }

if [[ "$RUNTIME" != "ros2" ]]; then
  echo "ROS1 waypoint dispatch is not wired yet (runtime=${RUNTIME})"
  exit 1
fi

MSG="$(python3 - "$WP_FILE" <<'PY'
import sys, yaml
wp = yaml.safe_load(open(sys.argv[1]))['waypoints']
items = ", ".join(
    "{x: %s, y: %s, yaw: %s, dwell_sec: %s, name: '%s'}" % (
        w['x'], w['y'], w.get('yaw', 0.0), w.get('dwell_sec', 0.0), w['name'])
    for w in wp)
print("{waypoints: [%s], loop: false, temporary: false}" % items)
PY
)"

echo "[send_waypoints] mission: ${MSG}"
docker exec "${CONTAINER}" bash -lc "
  source /opt/ros/jazzy/setup.bash && source /ons2/ros2_ws/install/setup.bash && \
  ros2 service call /waypoint_manager/start_mission nav_interfaces/srv/StartMission \"${MSG}\""

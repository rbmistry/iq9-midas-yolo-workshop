#!/usr/bin/env bash
set -euo pipefail

# Launch the MiDaS+YOLO parallel-inference demo using the on-board MIPI CSI
# camera (Sony IMX577, qrb_ros_camera) instead of a USB webcam.
#
# Why a separate script instead of a video_device:= override on
# run_usb_demo.sh: the CSI path uses a different ROS graph. The camera is a
# composable qrb_ros_camera::CameraNode fused into the SAME component
# container as the inference/fusion nodes (sample_midas_yolo_parallel
# launch_with_qrb_ros_camera.py), publishing /cam0_stream1, instead of a
# separate usb_cam process publishing /image_raw. Reusing run_usb_demo.sh
# would require a camera_device-style flag that doesn't apply to CSI, so a
# second thin wrapper keeps each script's logic honest about what it drives.
#
# Prerequisite: the MIPI CSI camera must be enumerated by the Qualcomm
# camera stack (`systemctl is-active cam-server`) and visible as logical
# camera 0. Verify with:
#   sudo systemctl status cam-server
#   sudo dmesg | grep -i cam_sensor_match_id   # confirms sensor id/model

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSHOP_ROOT="$(cd "$REPO_ROOT/.." && pwd)"
DEMO_REPO_DIR="${DEMO_REPO_DIR:-$WORKSHOP_ROOT/qrb_ros_samples}"

if [ ! -f "$DEMO_REPO_DIR/install/local_setup.bash" ]; then
  echo "Built workspace not found: $DEMO_REPO_DIR/install/local_setup.bash" >&2
  echo "Run ./scripts/prepare_demo_workspace.sh first." >&2
  exit 1
fi

if ! systemctl is-active --quiet cam-server; then
  echo "WARN: cam-server.service is not active; the CSI camera node will likely fail to open camera_id 0." >&2
  echo "      Check: sudo systemctl status cam-server" >&2
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$DEMO_REPO_DIR/install/local_setup.bash"
set -u

WEB_VIEWER="${WEB_VIEWER:-1}"
WEB_VIEWER_PORT="${WEB_VIEWER_PORT:-8080}"
WEB_VIEWER_ADDRESS="${WEB_VIEWER_ADDRESS:-0.0.0.0}"
WEB_VIEWER_TOPIC="${WEB_VIEWER_TOPIC:-/midas_yolo_overlay}"
WEB_VIEWER_QOS_PROFILE="${WEB_VIEWER_QOS_PROFILE:-sensor_data}"
WEB_VIEWER_LOG="${WEB_VIEWER_LOG:-/tmp/iq9_web_video_server.log}"
WEB_DASHBOARD="${WEB_DASHBOARD:-1}"
WEB_DASHBOARD_PORT="${WEB_DASHBOARD_PORT:-8081}"
WEB_DASHBOARD_LOG="${WEB_DASHBOARD_LOG:-/tmp/iq9_web_dashboard.log}"
# CSI camera publishes /cam0_stream1 (not /image_raw like usb_cam) at
# RELIABLE QoS (same as the USB path's default profile).
WEB_CAMERA_TOPIC="${WEB_CAMERA_TOPIC:-/cam0_stream1}"
WEB_CAMERA_QOS_PROFILE="${WEB_CAMERA_QOS_PROFILE:-default}"
WEB_DEPTH_TOPIC="${WEB_DEPTH_TOPIC:-/midas_depth_gray}"
web_viewer_pid=""
web_dashboard_pid=""

first_lan_address() {
  local ip
  for ip in $(hostname -I 2>/dev/null || true); do
    echo "$ip"
    return
  done
  hostname -s 2>/dev/null || echo localhost
}

stop_web_viewer() {
  if [ -n "$web_dashboard_pid" ] && kill -0 "$web_dashboard_pid" 2>/dev/null; then
    kill "$web_dashboard_pid" 2>/dev/null || true
    wait "$web_dashboard_pid" 2>/dev/null || true
  fi
  if [ -n "$web_viewer_pid" ] && kill -0 "$web_viewer_pid" 2>/dev/null; then
    kill "$web_viewer_pid" 2>/dev/null || true
    wait "$web_viewer_pid" 2>/dev/null || true
  fi
}
trap stop_web_viewer EXIT INT TERM

start_web_viewer() {
  if [ "$WEB_VIEWER" = "0" ]; then
    echo "Web viewer disabled by WEB_VIEWER=0."
    return
  fi

  if ! ros2 pkg prefix web_video_server >/dev/null 2>&1; then
    echo "WARN: web_video_server is not installed; run \"$REPO_ROOT/scripts/setup_iq9_workshop.sh\"." >&2
    return
  fi

  ros2 run web_video_server web_video_server \
    --ros-args \
    -p port:="$WEB_VIEWER_PORT" \
    -p address:="$WEB_VIEWER_ADDRESS" \
    >"$WEB_VIEWER_LOG" 2>&1 &
  web_viewer_pid=$!
  sleep 1
  if ! kill -0 "$web_viewer_pid" 2>/dev/null; then
    echo "WARN: web_video_server failed to start; see $WEB_VIEWER_LOG" >&2
    web_viewer_pid=""
    return
  fi

  viewer_host=$(first_lan_address)
  echo "Web viewer running: http://$viewer_host:$WEB_VIEWER_PORT/stream_viewer?topic=$WEB_VIEWER_TOPIC&qos_profile=$WEB_VIEWER_QOS_PROFILE"
  echo "Available ROS image topics: http://$viewer_host:$WEB_VIEWER_PORT/"
  echo "Web viewer log: $WEB_VIEWER_LOG"
}

start_web_dashboard() {
  if [ "$WEB_VIEWER" = "0" ] || [ "$WEB_DASHBOARD" = "0" ]; then
    return
  fi
  if [ -z "$web_viewer_pid" ]; then
    return
  fi

  python3 "$REPO_ROOT/scripts/iq9_web_dashboard.py" \
    --address "$WEB_VIEWER_ADDRESS" \
    --port "$WEB_DASHBOARD_PORT" \
    --video-port "$WEB_VIEWER_PORT" \
    --qos-profile "$WEB_VIEWER_QOS_PROFILE" \
    --camera-qos-profile "$WEB_CAMERA_QOS_PROFILE" \
    --camera-topic "$WEB_CAMERA_TOPIC" \
    --depth-topic "$WEB_DEPTH_TOPIC" \
    --overlay-topic "$WEB_VIEWER_TOPIC" \
    >"$WEB_DASHBOARD_LOG" 2>&1 &
  web_dashboard_pid=$!
  sleep 1
  if ! kill -0 "$web_dashboard_pid" 2>/dev/null; then
    echo "WARN: IQ9 web dashboard failed to start; see $WEB_DASHBOARD_LOG" >&2
    web_dashboard_pid=""
    return
  fi

  viewer_host=$(first_lan_address)
  echo "IQ9 live dashboard: http://$viewer_host:$WEB_DASHBOARD_PORT/"
  echo "IQ9 overview page:  http://$viewer_host:$WEB_DASHBOARD_PORT/overview"
  echo "Dashboard log: $WEB_DASHBOARD_LOG"
}

has_score_thresh_arg=0
for arg in "$@"; do
  if [[ "$arg" == score_thresh:=* ]]; then
    has_score_thresh_arg=1
    break
  fi
done

if [ "$has_score_thresh_arg" -eq 0 ]; then
  set -- "score_thresh:=0.5" "$@"
fi

start_web_viewer
start_web_dashboard

ros2 launch sample_midas_yolo_parallel launch_with_qrb_ros_camera.py "$@"
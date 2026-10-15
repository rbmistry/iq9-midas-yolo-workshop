#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSHOP_ROOT="$(cd "$REPO_ROOT/.." && pwd)"
DEMO_REPO_DIR="${DEMO_REPO_DIR:-$WORKSHOP_ROOT/qrb_ros_samples}"
NN_REPO_DIR="${NN_REPO_DIR:-$DEMO_REPO_DIR/qrb_ros_nn_inference}"
MODEL_SRC="$REPO_ROOT/models/midas_yolo_combined_int8_split.bin"
MODEL_DST=/opt/model/midas_yolo_combined_int8_split.bin

fail=0
check() {
  local name="$1"
  shift
  if "$@" >/tmp/iq9_check.out 2>/tmp/iq9_check.err; then
    echo "PASS: $name"
  else
    echo "FAIL: $name"
    cat /tmp/iq9_check.err >&2 || true
    fail=1
  fi
}

check "local model exists" test -f "$MODEL_SRC"
check "launch model exists" test -f "$MODEL_DST"
check "qrb_ros_samples checkout exists" test -d "$DEMO_REPO_DIR/.git"
check "qrb_ros_nn_inference source checkout exists" test -d "$NN_REPO_DIR/.git"

# shellcheck disable=SC1091
set +u
source /opt/ros/jazzy/setup.bash
set -u
if [ -f "$DEMO_REPO_DIR/install/local_setup.bash" ]; then
  # shellcheck disable=SC1091
  set +u
  source "$DEMO_REPO_DIR/install/local_setup.bash"
  set -u
else
  echo "FAIL: built workspace setup file missing: $DEMO_REPO_DIR/install/local_setup.bash" >&2
  fail=1
fi

check "sample package visible" ros2 pkg prefix sample_midas_yolo_parallel
check "web video server package visible" ros2 pkg prefix web_video_server
check "web dashboard script present" test -f "$REPO_ROOT/scripts/iq9_web_dashboard.py"
check "USB camera detector present" test -f "$REPO_ROOT/scripts/detect_usb_camera.py"
check "shared inference component visible" bash -lc 'ros2 component types | python3 -c "import sys; sys.exit(0 if any(\"QrbRosSharedInferenceNode\" in line for line in sys.stdin) else 1)"'
check "USB launch arguments parse" ros2 launch sample_midas_yolo_parallel launch_with_usb_cam.py --show-args

usb_camera=$(python3 "$REPO_ROOT/scripts/detect_usb_camera.py" --describe || true)
if [ -n "$usb_camera" ]; then
  echo "PASS: USB camera detected at $usb_camera"
  v4l2-ctl --list-devices 2>/dev/null || true
elif compgen -G '/dev/video*' >/dev/null; then
  echo "WARN: V4L video nodes exist, but no USB camera was identified"
  v4l2-ctl --list-devices 2>/dev/null || true
else
  echo "WARN: no /dev/video* devices found; plug the USB camera before the live demo"
fi

exit "$fail"

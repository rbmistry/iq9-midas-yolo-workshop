#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSHOP_ROOT="$(cd "$REPO_ROOT/.." && pwd)"
DEMO_REPO_DIR="${DEMO_REPO_DIR:-$WORKSHOP_ROOT/qrb_ros_samples}"
NN_REPO_DIR="${NN_REPO_DIR:-$DEMO_REPO_DIR/qrb_ros_nn_inference}"
MODEL_SRC="$REPO_ROOT/models/midas_yolo_combined_int8_split.bin"
MODEL_DST=/opt/model/midas_yolo_combined_int8_split.bin
SAMPLES_BRANCH=workshop-pr-429
NN_INFERENCE_REPO=https://github.com/samfreund-qc/qrb_ros_nn_inference.git
NN_INFERENCE_BRANCH=feat/multigraph-shared-context
NN_INFERENCE_COMMIT=cb91917e3ce82ce5e5075c747e33825a29a58036

if [ ! -f "$MODEL_SRC" ]; then
  echo "Missing model: $MODEL_SRC" >&2
  exit 1
fi

if [ ! -d "$DEMO_REPO_DIR/.git" ]; then
  git clone https://github.com/qualcomm-qrb-ros/qrb_ros_samples.git "$DEMO_REPO_DIR"
fi

git -C "$DEMO_REPO_DIR" fetch origin pull/429/head
git -C "$DEMO_REPO_DIR" checkout -B "$SAMPLES_BRANCH" FETCH_HEAD

if [ ! -d "$NN_REPO_DIR/.git" ]; then
  git clone --no-checkout "$NN_INFERENCE_REPO" "$NN_REPO_DIR"
else
  git -C "$NN_REPO_DIR" remote set-url origin "$NN_INFERENCE_REPO"
fi
git -C "$NN_REPO_DIR" fetch origin "$NN_INFERENCE_BRANCH"
git -C "$NN_REPO_DIR" checkout --detach "$NN_INFERENCE_COMMIT"
# The pinned fork currently has an invalid XML declaration (`<?xml version="1.1.1"?>`)
# in both ROS package manifests. Patch it before colcon discovery.
sed -i '1s/1\.1\.1/1.0/' \
  "$NN_REPO_DIR/qrb_inference_manager/package.xml" \
  "$NN_REPO_DIR/qrb_ros_nn_inference/package.xml"

sudo mkdir -p /opt/model
sudo cp "$MODEL_SRC" "$MODEL_DST"
sudo chmod 0644 "$MODEL_DST"

cd "$DEMO_REPO_DIR"
# shellcheck disable=SC1091
set +u
source /opt/ros/jazzy/setup.bash
set -u
if [ -f /usr/share/qirp-setup.sh ]; then
  # shellcheck disable=SC1091
  set +u
  source /usr/share/qirp-setup.sh || true
  set -u
fi
if ! ros2 pkg prefix web_video_server >/dev/null 2>&1; then
  echo "WARN: web_video_server is not installed; run \"$REPO_ROOT/scripts/setup_iq9_workshop.sh\" before launching the browser viewer." >&2
fi
if [ ! -f "$REPO_ROOT/scripts/iq9_web_dashboard.py" ]; then
  echo "WARN: web dashboard script missing: $REPO_ROOT/scripts/iq9_web_dashboard.py" >&2
fi
if [ ! -f "$REPO_ROOT/scripts/detect_usb_camera.py" ]; then
  echo "WARN: USB camera detector missing: $REPO_ROOT/scripts/detect_usb_camera.py" >&2
fi
colcon build --packages-select qrb_inference_manager qrb_ros_nn_inference sample_midas_yolo_parallel --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "Demo workspace ready at $DEMO_REPO_DIR"
echo "Model staged at $MODEL_DST"
echo "Shared inference source pinned at $NN_INFERENCE_REPO commit $NN_INFERENCE_COMMIT"

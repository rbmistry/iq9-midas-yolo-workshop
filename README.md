# IQ9 Parallel Depth + Segmentation Workshop

Run a real-time robotics perception demo on Qualcomm Dragonwing IQ-9075 EVK (**IQ9**) with ROS 2 Jazzy, QRB ROS, QNN/HTP inference, MiDaS depth, and YOLO11n segmentation.

This repository is the attendee-facing workshop package. Clone it on the IQ9, run the setup scripts, plug in a USB camera, and launch the demo.

## What you will build and run

```text
USB camera /image_raw
        ↓
MidasYoloFusionNode
  - preprocesses MiDaS depth tensor
  - preprocesses YOLO segmentation tensor
        ↓
QrbRosSharedInferenceNode
  - loads one shared QNN context binary
  - graph 0: midas
  - graph 1: yolov11_seg
        ↓
MidasYoloFusionNode
  - matches outputs by timestamp
  - decodes depth, detections, masks
        ↓
/midas_depth_map
/midas_depth_gray
/midas_yolo_overlay
```

The demo source comes from Qualcomm QRB ROS Samples PR #429: `sample_midas_yolo_parallel`. It also builds `qrb_ros_nn_inference` PR #93 because released `/opt/ros/jazzy` packages do not yet include `QrbRosSharedInferenceNode`.

## Why this is a QRB ROS workshop

QRB ROS keeps the application in normal ROS 2 concepts while exposing Qualcomm robotics acceleration paths.

| Robotics problem | Stock ROS 2 default | QRB ROS / IQ9 advantage |
| --- | --- | --- |
| Neural-network inference | CPU or generic GPU unless you wire QNN yourself | `qrb_ros_nn_inference` runs model artifacts through QNN / Hexagon HTP |
| Camera frame movement | `sensor_msgs::Image` payload copies between nodes | QRB transport can pass DMA-buf fds for hardware-to-hardware paths |
| Full perception pipeline | Manually wire camera, preprocess, inference, postprocess, topics | `qrb_ros_samples` gives accelerator-aware reference launch files |
| Evaluation path | Start from a blank ROS graph | Start from a working sample, then replace model/camera/postprocess as needed |

This workshop uses a USB camera for accessibility. Production IQ9 camera paths can move toward `qrb_ros_camera` and `qrb_ros_transport` for Qualcomm camera + DMA-buf workflows.

## Repository contents

```text
models/midas_yolo_combined_int8_split.bin   # included QNN context binary
scripts/setup_iq9_workshop.sh               # install ROS/QIRP/QRB ROS dependencies
scripts/prepare_demo_workspace.sh           # fetch PR #429 + qrb_ros_nn_inference PR #93, stage model, build overlay
scripts/check_demo_prereqs.sh               # verify model, ROS packages, launch args, camera nodes
scripts/detect_usb_camera.py                # pick the real USB capture node across /dev/video* enumeration changes
scripts/iq9_web_dashboard.py                # live camera/depth/overlay dashboard
scripts/run_usb_demo.sh                     # launch the USB-camera demo plus browser web viewer
scripts/run_csi_demo.sh                     # launch the on-board MIPI CSI camera demo plus browser web viewer
```

No slide/runbook docs are included in this attendee repo.

## Requirements

- Qualcomm Dragonwing IQ-9075 EVK / IQ9.
- Ubuntu 24.04 / Qualcomm Ubuntu image.
- ROS 2 Jazzy apt repository access.
- Internet access for apt and GitHub during setup.
- USB camera plugged into the IQ9 before the live demo.

## Quick start

Clone the workshop repository on the IQ9:

```bash
cd /home/ubuntu/workshop
git clone <WORKSHOP_REPO_URL> iq9-midas-yolo-workshop
cd iq9-midas-yolo-workshop
```

Install dependencies:

```bash
./scripts/setup_iq9_workshop.sh
```

Prepare the QRB ROS sample workspace, pin the shared-inference dependency, and build the overlay:

```bash
./scripts/prepare_demo_workspace.sh
```

Check readiness:

```bash
./scripts/check_demo_prereqs.sh
```

Run the USB-camera demo:

```bash
./scripts/run_usb_demo.sh
```

The helper auto-detects the first USB-backed `/dev/video*` node. To override it manually:

```bash
./scripts/run_usb_demo.sh video_device:=/dev/video2
```

The helper script is equivalent to sourcing ROS, sourcing the built overlay workspace, selecting the USB camera, setting the default YOLO score threshold to `0.5`, starting `web_video_server`, starting the IQ9 dashboard website, and running `ros2 launch sample_midas_yolo_parallel launch_with_usb_cam.py`.

When the demo starts, it prints a dashboard URL like:

```text
http://<IQ9_LAN_IP>:8081/
```

Open that URL from any machine on the same network to see a live website with three streams: raw camera (`/image_raw`), MiDaS depth (`/midas_depth_gray`), and MiDaS + YOLO overlay (`/midas_yolo_overlay`). The dashboard uses default/reliable QoS for the `usb_cam` camera stream and `sensor_data` QoS for MiDaS/YOLO output streams. The underlying stream server is still available at:

```text
http://<IQ9_LAN_IP>:8080/
```

Optional web-viewer overrides:

```bash
WEB_VIEWER=0 ./scripts/run_usb_demo.sh                         # disable HTTP output
WEB_DASHBOARD_PORT=8082 ./scripts/run_usb_demo.sh              # change the dashboard website port
WEB_VIEWER_PORT=8090 ./scripts/run_usb_demo.sh                 # change the ROS image stream port
WEB_CAMERA_TOPIC=/image_raw ./scripts/run_usb_demo.sh          # change the camera panel topic
WEB_CAMERA_QOS_PROFILE=sensor_data ./scripts/run_usb_demo.sh  # change the camera stream QoS query
WEB_DEPTH_TOPIC=/midas_depth_gray ./scripts/run_usb_demo.sh    # change the depth panel topic
WEB_VIEWER_TOPIC=/midas_yolo_overlay ./scripts/run_usb_demo.sh # change the overlay panel topic
WEB_VIEWER_QOS_PROFILE=default ./scripts/run_usb_demo.sh       # change stream QoS query
./scripts/run_usb_demo.sh score_thresh:=0.25                    # override YOLO score threshold
```

## Running the demo with the on-board MIPI CSI camera

The IQ9 EVK also exposes an on-board MIPI CSI camera (Sony IMX577) through the Qualcomm camera stack (`cam-server.service` + `qrb_ros_camera`), separate from any USB webcam. Use it instead of `run_usb_demo.sh` when a CSI camera module is attached:

```bash
./scripts/run_csi_demo.sh
```

Confirm the CSI camera is recognized before launching:

```bash
sudo systemctl status cam-server            # should be "active (running)"
sudo dmesg | grep -i cam_sensor_match_id    # should report the sensor id/name, e.g. cmk_imx577
```

`run_csi_demo.sh` is a thin wrapper, mirroring `run_usb_demo.sh`'s behavior (ROS/overlay sourcing, YOLO score-threshold default of `0.5`, `web_video_server`, and the IQ9 dashboard website), but runs `ros2 launch sample_midas_yolo_parallel launch_with_qrb_ros_camera.py` instead. That launch file composes the `qrb_ros_camera::CameraNode` directly into the same component container as the inference and fusion nodes, publishing the raw feed on `/cam0_stream1` (not `/image_raw`) at 640x480/30fps. The dashboard is pre-wired to this topic by default (`WEB_CAMERA_TOPIC=/cam0_stream1`), so no manual override is required:

```bash
./scripts/run_csi_demo.sh score_thresh:=0.25   # override YOLO score threshold, same as the USB path
```

The same `WEB_VIEWER`, `WEB_DASHBOARD_PORT`, `WEB_VIEWER_PORT`, `WEB_DEPTH_TOPIC`, `WEB_VIEWER_TOPIC`, and QoS override environment variables documented above for `run_usb_demo.sh` apply to `run_csi_demo.sh` as well.

If the stream appears slow, check for a stale camera process left over from a prior run before assuming a hardware or model issue — a leaked capture node competing for the same camera/CPU resources is a common and easy-to-miss cause of degraded frame rate:

```bash
ps aux | grep -E 'usb_cam|ros2 launch|component_container'
# kill any PID from a demo you believed had already exited, then re-launch
```

## What attendees do vs what scripts hide

The workshop should stay command-light. Attendees run the scripts and inspect ROS; the scripts handle slow or typo-prone plumbing.

| Attendees do | Scripts hide |
| --- | --- |
| Run `setup_iq9_workshop.sh` | Full apt package list, ROS apt key setup, Qualcomm PPA setup, QIRP setup script sourcing. |
| Run `prepare_demo_workspace.sh` | PR checkout, `qrb_ros_nn_inference` PR #93 pin, XML manifest patch, model copy to `/opt/model`, colcon package selection. |
| Run `check_demo_prereqs.sh` | Exact package/component/camera checks. |
| Run `run_usb_demo.sh`, open the printed dashboard URL, and inspect ROS topics | ROS environment sourcing, robust USB camera capture-node selection, dashboard website, YOLO score-threshold default, and HTTP image streaming. |

The useful live learning moments are the Qualcomm PPA/QIRP concept, ROS overlay build concept, readiness checks, and the running ROS graph. The fork pin, XML patch, and model staging are necessary plumbing, not attendee exercises.

## Live facilitation pattern

For the workshop, start each long-running command first, then explain it while it runs:

| Start this command | Talk track while it runs |
| --- | --- |
| `./scripts/setup_iq9_workshop.sh` | Board prep: Qualcomm PPAs, ROS 2 Jazzy tools, QIRP/QRB ROS packages, QNN/camera/runtime dependencies, web video server. |
| `./scripts/prepare_demo_workspace.sh` | ROS overlay: QRB ROS Samples PR #429 plus pinned `qrb_ros_nn_inference` PR #93 for `QrbRosSharedInferenceNode`, model staging, and `colcon build`. |
| `./scripts/check_demo_prereqs.sh` | Demo gate: model exists, packages are visible, shared inference component and web video server are registered, launch args parse, USB camera is detected. |
| `./scripts/run_usb_demo.sh` | Launch wrapper: source ROS/overlay, auto-detect the real USB capture node even when `/dev/video*` ordering changes, default YOLO score threshold to `0.5`, start the live dashboard website, start the ROS image stream server, and run the sample; then inspect the visual raw/depth/overlay outputs plus ROS nodes/topics/rates in a second terminal. |

## Inspect the running ROS graph

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/workshop/qrb_ros_samples/install/local_setup.bash
ros2 topic list
ros2 topic hz /midas_yolo_overlay
```

Expected demo topics include:

- `/image_raw`
- `/midas_inference_input_tensor`
- `/yolo_seg_inference_input_tensor`
- `/midas_inference_output_tensor`
- `/yolo_seg_inference_output_tensor`
- `/midas_depth_map`
- `/midas_depth_gray`
- `/midas_yolo_overlay`

## View the output in a browser

`run_usb_demo.sh` starts two HTTP services by default:

- IQ9 dashboard website on `0.0.0.0:8081`, showing `/image_raw`, `/midas_depth_gray`, and `/midas_yolo_overlay`.
- ROS 2 `web_video_server` on `0.0.0.0:8080`, serving the underlying MJPEG streams.

Use the printed dashboard URL to watch the raw camera, MiDaS depth, and MiDaS + YOLO overlay from a laptop on the same network as the IQ9.

If the dashboard cannot connect or a panel opens without frames:

1. Confirm the laptop and IQ9 are on the same network.
2. Open `http://<IQ9_LAN_IP>:8080/` and check that `/image_raw`, `/midas_depth_gray`, and `/midas_yolo_overlay` appear.
3. Keep the default camera QoS for `/image_raw`; use `qos_profile=sensor_data` for MiDaS/YOLO output image publishers.
4. Check the logs printed by `run_usb_demo.sh` (defaults: `/tmp/iq9_web_dashboard.log` and `/tmp/iq9_web_video_server.log`).

## Model artifact

The model is included in this repo:

```text
models/midas_yolo_combined_int8_split.bin
```

`prepare_demo_workspace.sh` copies it to the launch-file default:

```text
/opt/model/midas_yolo_combined_int8_split.bin
```

The binary contains two QNN graphs in one shared context:

| Graph | Purpose |
| --- | --- |
| `midas` | MiDaS depth estimation |
| `yolov11_seg` | YOLO11n segmentation |


## Source overlays

`prepare_demo_workspace.sh` builds these source packages into `/home/ubuntu/workshop/qrb_ros_samples/install`:

| Package | Source | Reason |
| --- | --- | --- |
| `sample_midas_yolo_parallel` | `qualcomm-qrb-ros/qrb_ros_samples` PR #429 | Workshop demo package |
| `qrb_inference_manager` / `qrb_ros_nn_inference` | `samfreund-qc/qrb_ros_nn_inference` commit `cb91917e3ce82ce5e5075c747e33825a29a58036` from PR #93 | Provides `qrb_ros::nn_inference::QrbRosSharedInferenceNode` |

Do not run this demo against only the released `ros-jazzy-qrb-ros-nn-inference` package; that package currently registers `QrbRosInferenceNode` only.

## Troubleshooting

| Symptom | Check | Fix |
| --- | --- | --- |
| Package not found | `ros2 pkg prefix sample_midas_yolo_parallel` | Re-run `./scripts/prepare_demo_workspace.sh` |
| Model load fails | `ls -lh /opt/model/midas_yolo_combined_int8_split.bin` | Re-run prepare script; it stages the included model |
| No camera | `v4l2-ctl --list-devices` | Plug camera, then pass `video_device:=/dev/videoX` |
| No overlay topic | `ros2 topic list` | Confirm launch is still running and `/image_raw` exists |
| `Failed to find class ... QrbRosSharedInferenceNode` | `ros2 component types | grep QrbRosSharedInferenceNode` | Re-run `./scripts/prepare_demo_workspace.sh`; it builds qrb_ros_nn_inference PR #93 into the overlay |
| Low/no detections | Increase lighting, aim at common COCO-like objects, or adjust `score_thresh` |

## Upstream references

- Demo PR: https://github.com/qualcomm-qrb-ros/qrb_ros_samples/pull/429
- QRB ROS NN inference shared-context PR: https://github.com/qualcomm-qrb-ros/qrb_ros_nn_inference/pull/93
- QRB ROS overview: https://dragonwingdocs.qualcomm.com/Ubuntu/robotics-workflows/qrb-ros-overview
- QRB ROS NN inference: https://dragonwingdocs.qualcomm.com/Ubuntu/robotics-workflows/qrb-ros-nn-inference
- QRB ROS transport: https://dragonwingdocs.qualcomm.com/Ubuntu/robotics-workflows/qrb-ros-transport
- QRB ROS samples: https://dragonwingdocs.qualcomm.com/Ubuntu/robotics-workflows/qrb-ros-samples
- IQ-9075 flash flow: https://dragonwingdocs.qualcomm.com/Ubuntu/devices/iq9075-evk/update-software/flash-using-qualcomm-launcher
- IQ-9075 required packages: https://dragonwingdocs.qualcomm.com/Ubuntu/devices/iq9075-evk/Install_required_software_packages
- ROS / robotics setup: https://dragonwingdocs.qualcomm.com/Ubuntu/robotics-workflows/software-setup

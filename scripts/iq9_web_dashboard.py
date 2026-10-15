#!/usr/bin/env python3
"""Serve the IQ9 workshop browser dashboard.

Two pages, one stdlib-only HTTP server (no new dependencies):

  /           Live MJPEG dashboard (raw camera / MiDaS depth / YOLO overlay).
  /overview   Static "how this demo works" page: installed ROS 2 version,
              Ubuntu install steps, a categorized ROS node table, and a
              flowchart of the QRB ROS perception pipeline this demo runs.

Design notes
------------
- Zero added Python dependencies (stdlib ``http.server`` + f-strings only),
  matching the original single-page dashboard. Nothing to pip-install or
  go stale offline on an EVK with no internet access.
- All /overview content is generated from small Python data tables
  (``ROS_PLATFORM_INFO``, ``INSTALL_STEPS``, ``NODE_CATALOG``,
  ``PIPELINE_STAGES``) via one shared table-rendering helper, instead of
  hand-written HTML blocks. Updating the page means editing a Python list,
  not hunting for markup -- this is what keeps the page maintainable and
  easy to replicate/extend on another IQ9 EVK.
- The pipeline flowchart is plain HTML/CSS (flex boxes + arrows), not a
  JS diagram library, so it renders identically with zero internet access.
- Data snapshots are captured on-device (commands are documented next to
  each table below) rather than queried live, so the page loads instantly
  and still works before the ROS graph is launched.
"""

from __future__ import annotations

import argparse
import html
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import quote


DEFAULT_CAMERA_TOPIC = "/image_raw"
DEFAULT_DEPTH_TOPIC = "/midas_depth_gray"
DEFAULT_OVERLAY_TOPIC = "/midas_yolo_overlay"


# ---------------------------------------------------------------------------
# Static documentation data for the /overview page.
# Captured on a Dragonwing IQ-9075 EVK workshop image with:
#   cat /etc/os-release && apt-cache policy ros-jazzy-desktop && uname -a
# ---------------------------------------------------------------------------

ROS_PLATFORM_INFO = [
    ("ROS 2 distribution", "ROS 2 Jazzy Jalisco (ros-jazzy-desktop 0.11.0)"),
    ("Host OS", "Ubuntu 24.04.5 LTS (Noble Numbat)"),
    ("Kernel", "6.8.0-1080-qcom (aarch64, Qualcomm)"),
    ("Board", "Qualcomm Dragonwing IQ-9075 EVK (IQ9)"),
    ("Qualcomm ROS layer", "QIRP SDK via ppa:ubuntu-qcom-iot/qcom-ppa + "
                           "ppa:ubuntu-qcom-iot/qirp (provides ros-jazzy-qrb-ros-* "
                           "packages and QNN/Hexagon runtime libs)"),
]

# Condensed from scripts/setup_iq9_workshop.sh, which remains the source of
# truth; this table is documentation only.
INSTALL_STEPS = [
    ("1. Add the ROS 2 apt repository",
     "sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \\\n"
     "  -o /usr/share/keyrings/ros-archive-keyring.gpg\n"
     'echo "deb [arch=$(dpkg --print-architecture) '
     "signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] "
     'http://packages.ros.org/ros2/ubuntu $(. /etc/os-release \u0026\u0026 echo "$UBUNTU_CODENAME") main" \\\n'
     "  | sudo tee /etc/apt/sources.list.d/ros2.list"),
    ("2. Add the Qualcomm PPAs",
     "sudo add-apt-repository -y ppa:ubuntu-qcom-iot/qcom-ppa\n"
     "sudo add-apt-repository -y ppa:ubuntu-qcom-iot/qirp\n"
     "sudo apt update"),
    ("3. Install the QIRP SDK and ROS 2 Jazzy desktop",
     "sudo apt install -y qirp-sdk ros-jazzy-desktop \\\n"
     "  python3-colcon-common-extensions python3-rosdep python3-argcomplete"),
    ("4. Install QRB ROS, vision, and QNN runtime packages",
     "sudo apt install -y \\\n"
     "  ros-jazzy-qrb-ros-camera ros-jazzy-qrb-ros-nn-inference \\\n"
     "  ros-jazzy-qrb-ros-tensor-list-msgs ros-jazzy-image-publisher \\\n"
     "  ros-jazzy-cv-bridge ros-jazzy-usb-cam ros-jazzy-image-transport-plugins \\\n"
     "  ros-jazzy-web-video-server libqnn-dev libqnn1 \\\n"
     "  libtensorflow-lite-c-qcom1 libtensorflow-lite-qcom-dev v4l-utils"),
    ("5. Source the Qualcomm ROS environment hook (every new shell)",
     "source /opt/ros/jazzy/setup.bash\n"
     "source /usr/share/qirp-setup.sh"),
]
INSTALL_SCRIPT_HINT = "Scripted equivalent of all steps above: scripts/setup_iq9_workshop.sh"

# Node/component inventory snapshot captured on-device with:
#   source /opt/ros/jazzy/setup.bash \u0026\u0026 source /usr/share/qirp-setup.sh \u0026\u0026 \
#     source /home/ubuntu/qrb_ros_samples/install/local_setup.bash
#   ros2 pkg executables     # standalone node executables
#   ros2 component types     # composable node / component classes
#
# used=True marks the nodes this specific demo (run_usb_demo.sh ->
# launch_with_usb_cam.py) actually launches at runtime.
NODE_CATALOG = [
    ("AI Inference (QNN / Hexagon NPU)",
     "Runs compiled .bin/.dlc model graphs on the on-chip NPU accelerator.",
     [
        ("qrb_ros_nn_inference", "QrbRosSharedInferenceNode", "component", True),
        ("qrb_ros_nn_inference", "QrbRosInferenceNode", "component", False),
        ("qrb_ros_yolo_process", "YoloDet/SegPostProcessNode", "component", False),
        ("qrb_ros_yolo_process", "YoloDet/SegOverlayNode", "component", False),
        ("sample_midas_yolo_parallel", "MidasYoloFusionNode", "component", True),
        ("sample_resnet101", "qrb_ros_resnet101(_posprocess)", "node", False),
        ("sample_depth_estimation", "depth_estimation_node", "node", False),
        ("sample_face_detection", "qrb_ros_face_detector", "node", False),
        ("sample_hand_detection", "qrb_ros_hand_detector", "node", False),
        ("sample_hrnet_pose_estimation", "sample_hrnet_pose_estimation", "node", False),
     ]),
    ("Camera \u0026 Vision I/O",
     "Captures or synthesizes the frames that feed inference.",
     [
        ("usb_cam", "UsbCamNode", "node", True),
        ("qrb_ros_camera", "CameraNode / TestNode", "component", False),
        ("sample_midas_yolo_parallel", "FastImagePublisherNode", "component", False),
        ("qrb_ros_colorspace_convert", "ColorspaceConvertNode", "component", False),
        ("qrb_ros_cv_tensor_common_process", "CvTensorCommonProcessNode", "component", False),
        ("qrb_ros_video", "Decoder/Encoder/ImageReader/Writer", "component", False),
        ("camera_ros, orbbec_camera", "generic V4L2/libcamera/Orbbec drivers", "mixed", False),
     ]),
    ("Navigation \u0026 SLAM",
     "Full mobile-robot stack preinstalled on this image; not used by this perception demo.",
     [
        ("slam_toolbox", "Sync/Async/Lifelong SLAM", "component", False),
        ("cartographer_ros", "cartographer_node", "node", False),
        ("nav2_bt_navigator, nav2_planner, nav2_controller", "Nav2 behavior stack", "component", False),
        ("nav2_amcl", "AmclNode", "component", False),
        ("nav2_map_server", "MapServer / MapSaver", "component", False),
        ("apriltag_ros", "AprilTagNode", "component", False),
     ]),
    ("Robot Motion \u0026 Manipulation",
     "AMR driving and arm pick-and-place samples.",
     [
        ("qrb_ros_amr", "qrb_ros_amr", "node", False),
        ("qrb_ros_follow_path", "qrb_ros_follow_path", "node", False),
        ("simulation_sample_pick_and_place", "qrb_ros_arm_pick_place", "node", False),
        ("simulation_sample_amr_simple_motion", "simple_motion", "node", False),
        ("moveit_ros_planning, moveit_ros_move_group", "MoveIt motion planning", "mixed", False),
     ]),
    ("Audio",
     "Speech capture/playback samples from the separate audio workshop track.",
     [
        ("qrb_ros_audio_common", "AudioCommonNode", "component", False),
        ("qrb_ros_audio_service", "AudioServer / AudioCommonClient", "component", False),
     ]),
    ("Sensors \u0026 Telemetry",
     "IMU, battery, and board health reporting.",
     [
        ("qrb_ros_imu", "ImuComponent / ImuListener", "component", False),
        ("qrb_ros_battery", "battery_node", "node", False),
        ("qrb_ros_system_monitor", "Cpu/Mem/Disk/Swap/Temp/Battery Monitor", "component", False),
     ]),
    ("Visualization \u0026 Streaming",
     "Gets ROS image topics into a browser for this demo.",
     [
        ("web_video_server", "WebVideoServer", "component", True),
        ("iq9_web_dashboard.py", "this dashboard process (stdlib http.server)", "script", True),
        ("rosbag2_transport", "Player / Recorder", "component", False),
     ]),
    ("Benchmark \u0026 Dev Tools",
     "Performance measurement plus stock ROS 2 tutorial/reference nodes.",
     [
        ("qrb_ros_benchmark", "QrbMonitorNode / QrbPlaybackNode", "component", False),
        ("ros2_benchmark", "DataLoader/Monitor/PlaybackNode", "component", False),
        ("demo_nodes_cpp, composition, examples_*", "stock ROS 2 tutorial nodes", "mixed", False),
     ]),
]

# Pipeline flowchart stages, top to bottom. Each stage is one or more
# parallel boxes; "arrow_label" documents the message/topic connecting a
# stage to the next one (empty string for the first stage).
PIPELINE_STAGES = [
    ("", ["usb_cam_node\n(sensor_msgs/Image)"]),
    ("/image_raw", ["midas_yolo_fusion_node\npreprocess \u2192 2 tensors"]),
    ("/midas_inference_input_tensor\n/yolo_seg_inference_input_tensor",
     ["shared_inference_node\n(QrbRosSharedInferenceNode)\nQNN context on Hexagon NPU\ngraph 0: midas   graph 1: yolov11_seg"]),
    ("/midas_inference_output_tensor\n/yolo_seg_inference_output_tensor",
     ["midas_yolo_fusion_node\nmatch by timestamp \u2192 decode depth + boxes/masks"]),
    ("", ["/midas_depth_map", "/midas_depth_gray", "/midas_yolo_overlay"]),
]
PIPELINE_CONTAINER_NOTE = (
    "shared_inference_node and midas_yolo_fusion_node run as ComposableNodes inside "
    "ONE component_container_mt process (\u201cparallel_inference_container\u201d) for "
    "zero-copy intra-process tensor transport. Only usb_cam_node is a separate process."
)


# ---------------------------------------------------------------------------
# Shared page shell
# ---------------------------------------------------------------------------

PAGE_STYLE = """
    :root { color-scheme: dark; --bg:#03070d; --panel:#0c1420; --line:#1c2a3a;
            --text:#e7edf5; --muted:#8fa3bb; --accent:#4fd1c5; --accent2:#f6ad55; }
    * { box-sizing: border-box; }
    body { margin: 0; background: var(--bg); color: var(--text);
           font-family: -apple-system, Segoe UI, Roboto, Helvetica, Arial, sans-serif; }
    nav { display: flex; gap: 1rem; align-items: center; padding: .75rem 1.25rem;
          background: var(--panel); border-bottom: 1px solid var(--line); position: sticky; top: 0; }
    nav a { color: var(--muted); text-decoration: none; font-weight: 600; font-size: .95rem;
            padding: .4rem .7rem; border-radius: 8px; }
    nav a.active, nav a:hover { color: var(--text); background: #13202f; }
    nav .brand { color: var(--accent); font-weight: 800; margin-right: .5rem; }
    main.wrap { max-width: 1100px; margin: 0 auto; padding: 1.5rem 1.25rem 3rem; }
    h1 { font-size: 1.5rem; margin: .2rem 0 .3rem; }
    h2 { font-size: 1.1rem; color: var(--accent); border-bottom: 1px solid var(--line);
         padding-bottom: .4rem; margin-top: 2.2rem; }
    p.lead { color: var(--muted); margin-top: 0; }
    table { width: 100%; border-collapse: collapse; margin: .75rem 0 1.5rem; font-size: .92rem; }
    th, td { text-align: left; padding: .5rem .6rem; border-bottom: 1px solid var(--line); vertical-align: top; }
    th { color: var(--muted); font-weight: 600; font-size: .8rem; text-transform: uppercase; letter-spacing: .03em; }
    tr.used td:first-child { border-left: 3px solid var(--accent2); }
    .badge { display: inline-block; font-size: .72rem; padding: .1rem .5rem; border-radius: 999px;
             background: #13202f; color: var(--muted); }
    .badge.used { background: #3a2a12; color: var(--accent2); }
    .cat-note { color: var(--muted); font-size: .85rem; margin: .2rem 0 .4rem; }
    pre { background: #06101c; border: 1px solid var(--line); border-radius: 10px;
          padding: .75rem .9rem; overflow-x: auto; font-size: .82rem; line-height: 1.5; }
    .step { margin-bottom: 1.1rem; }
    .step h3 { font-size: .95rem; margin: 0 0 .35rem; }
    .hint { color: var(--muted); font-size: .85rem; margin-top: -.5rem; }
    .flow { display: flex; flex-direction: column; align-items: center; gap: .3rem; margin: 1rem 0; }
    .flow-row { display: flex; gap: .6rem; justify-content: center; flex-wrap: wrap; }
    .flow-box { background: var(--panel); border: 1px solid var(--line); border-radius: 10px;
                padding: .6rem .8rem; font-size: .82rem; white-space: pre-line; text-align: center;
                min-width: 180px; }
    .flow-arrow { color: var(--accent); font-size: .78rem; text-align: center; white-space: pre-line; }
    .flow-note { color: var(--muted); font-size: .8rem; text-align: center; max-width: 640px; margin-top: .3rem; }
    .card { display: flex; align-items: center; justify-content: center; min-height: 30vh;
            background: #03070d; border-radius: 16px; overflow: hidden; }
    main.dash { display: grid; gap: .75rem; grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
                min-height: calc(100vh - 56px); padding: .75rem; box-sizing: border-box; }
    img.stream { display: block; width: 100%; height: 100%; min-height: 30vh; max-height: 100vh;
                 object-fit: contain; background: #03070d; }
"""


def render_nav(active: str) -> str:
    def link(path: str, label: str) -> str:
        cls = "active" if path == active else ""
        return f'<a class="{cls}" href="{path}">{html.escape(label)}</a>'

    return (
        '<nav><span class="brand">IQ9 Workshop</span>'
        + link("/", "Live Dashboard")
        + link("/overview", "Overview")
        + "</nav>"
    )


def render_key_value_table(rows: list[tuple[str, str]]) -> str:
    body = "".join(
        f"<tr><th style='width:220px'>{html.escape(k)}</th><td>{html.escape(v)}</td></tr>"
        for k, v in rows
    )
    return f"<table><tbody>{body}</tbody></table>"


def render_install_steps(steps: list[tuple[str, str]], hint: str) -> str:
    blocks = []
    for title, cmd in steps:
        blocks.append(
            f'<div class="step"><h3>{html.escape(title)}</h3>'
            f'<pre>{html.escape(cmd)}</pre></div>'
        )
    return "".join(blocks) + f'<p class="hint">{html.escape(hint)}</p>'


def render_node_catalog(catalog: list[tuple[str, str, list[tuple[str, str, str, bool]]]]) -> str:
    sections = []
    for category, note, rows in catalog:
        row_html = []
        for pkg, name, kind, used in rows:
            used_cls = "used" if used else ""
            used_badge = '<span class="badge used">used by this demo</span>' if used \
                else '<span class="badge">available</span>'
            row_html.append(
                f'<tr class="{used_cls}"><td>{html.escape(pkg)}</td>'
                f"<td>{html.escape(name)}</td><td>{html.escape(kind)}</td>"
                f"<td>{used_badge}</td></tr>"
            )
        sections.append(
            f"<h2>{html.escape(category)}</h2>"
            f'<p class="cat-note">{html.escape(note)}</p>'
            "<table><thead><tr><th>Package</th><th>Node / Component</th>"
            "<th>Kind</th><th>Status</th></tr></thead>"
            f"<tbody>{''.join(row_html)}</tbody></table>"
        )
    return "".join(sections)


def render_pipeline(stages: list[tuple[str, list[str]]], note: str) -> str:
    parts = ['<div class="flow">']
    for i, (arrow_label, boxes) in enumerate(stages):
        if arrow_label:
            parts.append(f'<div class="flow-arrow">\u2193\n{html.escape(arrow_label)}</div>')
        elif i > 0:
            parts.append('<div class="flow-arrow">\u2193</div>')
        row = "".join(f'<div class="flow-box">{html.escape(b)}</div>' for b in boxes)
        parts.append(f'<div class="flow-row">{row}</div>')
    parts.append(f'<p class="flow-note">{html.escape(note)}</p>')
    parts.append("</div>")
    return "".join(parts)


def stream_url(video_host: str, video_port: int, topic: str, qos_profile: str) -> str:
    return (
        f"http://{video_host}:{video_port}/stream"
        f"?topic={quote(topic)}&type=mjpeg&qos_profile={quote(qos_profile)}"
    )


def render_dashboard(
    *,
    request_host: str,
    video_port: int,
    default_qos_profile: str,
    camera_qos_profile: str,
    camera_topic: str,
    depth_topic: str,
    overlay_topic: str,
) -> bytes:
    video_host = request_host.split(":", 1)[0] or "127.0.0.1"
    streams = [
        (camera_topic, camera_qos_profile),
        (depth_topic, default_qos_profile),
        (overlay_topic, default_qos_profile),
    ]
    cards = []
    for topic, stream_qos_profile in streams:
        src = stream_url(video_host, video_port, topic, stream_qos_profile)
        cards.append(
            f'<section class="card"><img class="stream" src="'
            f'{html.escape(src, quote=True)}" alt="" /></section>'
        )

    body = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>IQ9 Live Dashboard</title>
  <style>{PAGE_STYLE}</style>
</head>
<body>
  {render_nav("/")}
  <main class="dash">
    {''.join(cards)}
  </main>
</body>
</html>
"""
    return body.encode("utf-8")


def render_overview() -> bytes:
    body = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>IQ9 Workshop Overview</title>
  <style>{PAGE_STYLE}</style>
</head>
<body>
  {render_nav("/overview")}
  <main class="wrap">
    <h1>IQ9 MiDaS + YOLO Workshop \u2014 Overview</h1>
    <p class="lead">What is installed, what ROS nodes exist on this board, and how
       this specific demo wires camera \u2192 NPU inference \u2192 fused output.</p>

    <h2>ROS 2 Platform</h2>
    {render_key_value_table(ROS_PLATFORM_INFO)}

    <h2>Ubuntu Installation Steps</h2>
    {render_install_steps(INSTALL_STEPS, INSTALL_SCRIPT_HINT)}

    <h2>QRB ROS Perception Pipeline (this demo)</h2>
    {render_pipeline(PIPELINE_STAGES, PIPELINE_CONTAINER_NOTE)}

    <h2>ROS Nodes Available On This Board (by category)</h2>
    {render_node_catalog(NODE_CATALOG)}
  </main>
</body>
</html>
"""
    return body.encode("utf-8")


class DashboardHandler(BaseHTTPRequestHandler):
    server: "DashboardServer"

    def _send_html(self, content: bytes) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(content)

    def do_GET(self) -> None:  # noqa: N802 - http.server API
        if self.path in ("/", "/index.html"):
            self._send_html(render_dashboard(
                request_host=self.headers.get("Host", "127.0.0.1"),
                video_port=self.server.video_port,
                default_qos_profile=self.server.default_qos_profile,
                camera_qos_profile=self.server.camera_qos_profile,
                camera_topic=self.server.camera_topic,
                depth_topic=self.server.depth_topic,
                overlay_topic=self.server.overlay_topic,
            ))
            return

        if self.path in ("/overview", "/overview.html"):
            self._send_html(render_overview())
            return

        self.send_error(404)

    def log_message(self, format: str, *args: object) -> None:
        print(f"{self.address_string()} - {format % args}")


class DashboardServer(ThreadingHTTPServer):
    def __init__(
        self,
        server_address: tuple[str, int],
        handler_class: type[BaseHTTPRequestHandler],
        *,
        video_port: int,
        default_qos_profile: str,
        camera_qos_profile: str,
        camera_topic: str,
        depth_topic: str,
        overlay_topic: str,
    ) -> None:
        super().__init__(server_address, handler_class)
        self.video_port = video_port
        self.default_qos_profile = default_qos_profile
        self.camera_qos_profile = camera_qos_profile
        self.camera_topic = camera_topic
        self.depth_topic = depth_topic
        self.overlay_topic = overlay_topic


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serve the IQ9 web dashboard")
    parser.add_argument("--address", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--video-port", type=int, default=8080)
    parser.add_argument("--qos-profile", default="sensor_data")
    parser.add_argument("--camera-qos-profile", default="default")
    parser.add_argument("--camera-topic", default=DEFAULT_CAMERA_TOPIC)
    parser.add_argument("--depth-topic", default=DEFAULT_DEPTH_TOPIC)
    parser.add_argument("--overlay-topic", default=DEFAULT_OVERLAY_TOPIC)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    server = DashboardServer(
        (args.address, args.port),
        DashboardHandler,
        video_port=args.video_port,
        default_qos_profile=args.qos_profile,
        camera_qos_profile=args.camera_qos_profile,
        camera_topic=args.camera_topic,
        depth_topic=args.depth_topic,
        overlay_topic=args.overlay_topic,
    )
    print(f"IQ9 dashboard serving on http://{args.address}:{args.port}/", flush=True)
    print(f"IQ9 overview page:     http://{args.address}:{args.port}/overview", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
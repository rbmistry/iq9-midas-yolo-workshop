#!/usr/bin/env python3
"""Convert the on-board MIPI CSI camera's NV12 frames to BGR8 in software.

Why this exists
---------------
The CSI camera path (``qrb_ros_camera::CameraNode``) publishes
``sensor_msgs/Image`` frames encoded as ``nv12``. ``web_video_server``'s
``cv_bridge``-based MJPEG streamer only understands encodings OpenCV's
``cv_bridge`` recognizes directly (``rgb8``, ``bgr8``, ``mono8``, ...), so the
raw camera panel on the dashboard stays blank with
``cv_bridge exception: Unrecognized image encoding [nv12]`` in the
``web_video_server`` log.

Qualcomm ships ``qrb_ros_colorspace_convert`` for exactly this conversion,
but its NV12->RGB8 path is implemented with an OpenGL ES / EGL shader and
requires a running Wayland/X display server (``weston``). On a browser-only
workshop EVK there is no reason to start a full compositor just to flip
pixel formats for a debug stream, so this script does the same conversion
in software with OpenCV's native NV12 colorspace enum
(``cv2.COLOR_YUV2BGR_NV12``), which has no display dependency and is fast
enough at 640x480/30fps on the IQ9's CPU.

Design notes
------------
- Single-purpose, stdlib + numpy + OpenCV + rclpy only -- all already
  installed as part of the ROS 2 Jazzy + QIRP SDK setup, so this adds no
  new dependencies.
- Subscribes once, republishes once; no buffering/queueing beyond the
  default QoS depth, so latency stays low and the implementation stays
  easy to read and replicate on another IQ9 EVK.
- Non-``nv12`` input frames are republished unchanged (so changing the
  camera config later doesn't silently break the pipeline), with a single
  warning instead of warning every frame.
"""

from __future__ import annotations

import argparse

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class Nv12ToBgr8Node(Node):
    def __init__(self, input_topic: str, output_topic: str) -> None:
        super().__init__("csi_nv12_to_bgr8")
        self._warned_bad_encoding = False
        self._pub = self.create_publisher(Image, output_topic, 10)
        self._sub = self.create_subscription(Image, input_topic, self._on_image, 10)
        self.get_logger().info(f"Converting {input_topic} (nv12) -> {output_topic} (bgr8)")

    def _on_image(self, msg: Image) -> None:
        if msg.encoding != "nv12":
            if not self._warned_bad_encoding:
                self.get_logger().warning(
                    f"Expected encoding 'nv12' on {self._sub.topic_name}, got "
                    f"'{msg.encoding}'; republishing unchanged."
                )
                self._warned_bad_encoding = True
            self._pub.publish(msg)
            return

        height, width = msg.height, msg.width
        yuv = np.frombuffer(msg.data, dtype=np.uint8).reshape((height * 3 // 2, width))
        bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)

        out = Image()
        out.header = msg.header
        out.height = height
        out.width = width
        out.encoding = "bgr8"
        out.is_bigendian = 0
        out.step = width * 3
        out.data = bgr.tobytes()
        self._pub.publish(out)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-topic", default="/cam0_stream1")
    parser.add_argument("--output-topic", default="/cam0_stream1_rgb")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = Nv12ToBgr8Node(args.input_topic, args.output_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
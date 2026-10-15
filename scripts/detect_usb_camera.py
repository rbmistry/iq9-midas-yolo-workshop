#!/usr/bin/env python3
"""Select the USB V4L2 capture node for the IQ9 workshop demo.

USB cameras often enumerate more than one /dev/video* node. For UVC devices,
one node is usually the real image capture stream and another can be metadata.
This helper picks the USB node whose V4L2 device capabilities include Video
Capture and exclude metadata-only / memory-to-memory devices.
"""

from __future__ import annotations

import argparse
import subprocess
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Candidate:
    path: Path
    name: str
    device_path: str
    index: int
    info: str
    has_video_capture: bool
    metadata_only: bool
    memory_to_memory: bool

    @property
    def score(self) -> tuple[int, int, int, str]:
        # Lowest tuple wins. Prefer true capture devices, non-metadata devices,
        # the first interface of a multi-node UVC camera, then stable node order.
        return (
            0 if self.has_video_capture else 1,
            1 if self.metadata_only or self.memory_to_memory else 0,
            self.index,
            self.path.name,
        )


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def v4l2_info(device: Path) -> str:
    try:
        return subprocess.run(
            ["v4l2-ctl", "-d", str(device), "--info"],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=2,
        ).stdout
    except (OSError, subprocess.SubprocessError):
        return ""


def device_caps_block(info: str) -> str:
    marker = "Device Caps"
    if marker not in info:
        return info
    return info[info.index(marker) :]


def candidate_for(node: Path) -> Candidate | None:
    device_path = str((node / "device").resolve())
    name = read_text(node / "name")
    if "/usb" not in device_path or not name:
        return None

    try:
        index = int(read_text(node / "index") or "99")
    except ValueError:
        index = 99

    path = Path("/dev") / node.name
    info = v4l2_info(path)
    caps = device_caps_block(info)
    has_video_capture = "Video Capture" in caps and "Memory-to-Memory" not in caps
    memory_to_memory = "Memory-to-Memory" in caps
    metadata_only = "Metadata Capture" in caps and "Video Capture" not in caps

    # If v4l2-ctl is unavailable or returns no capability text, fall back to the
    # usual UVC convention: interface index 0 is the image stream.
    if not info:
        has_video_capture = index == 0
        metadata_only = index != 0

    return Candidate(
        path=path,
        name=name,
        device_path=device_path,
        index=index,
        info=info,
        has_video_capture=has_video_capture,
        metadata_only=metadata_only,
        memory_to_memory=memory_to_memory,
    )


def find_camera() -> Candidate | None:
    candidates = []
    for node in sorted(Path("/sys/class/video4linux").glob("video*")):
        candidate = candidate_for(node)
        if candidate is not None:
            candidates.append(candidate)

    usable = [candidate for candidate in candidates if candidate.has_video_capture and not candidate.metadata_only]
    if usable:
        return sorted(usable, key=lambda candidate: candidate.score)[0]
    if candidates:
        return sorted(candidates, key=lambda candidate: candidate.score)[0]
    return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Detect the USB camera video node")
    parser.add_argument("--describe", action="store_true", help="print path plus camera details")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    camera = find_camera()
    if camera is None:
        return 1

    if args.describe:
        print(f"{camera.path} ({camera.name}, index {camera.index})")
    else:
        print(camera.path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

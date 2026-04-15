import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, os.path.abspath("src"))

from dummy_robot import OpenCVCamera, extract_laser_line, generate_synthetic_laser_image


OUTPUT_DIR = Path("test_outputs/opencv_camera_smoke_test")
VIDEO_PATH = OUTPUT_DIR / "synthetic_scan.mp4"


def build_test_video() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(VIDEO_PATH), cv2.VideoWriter_fourcc(*"mp4v"), 10.0, (320, 240))
    if not writer.isOpened():
        raise RuntimeError(f"Failed to create test video: {VIDEO_PATH}")
    for index in range(12):
        rgb = generate_synthetic_laser_image(width=320, height=240, intercept=100 + index * 4, slope=0.05)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        writer.write(bgr)
    writer.release()


def main() -> None:
    build_test_video()
    camera = OpenCVCamera(name="smoke_camera", source=str(VIDEO_PATH), warmup_frames=0)
    summaries = []
    with camera:
        for frame_index in range(5):
            frame = camera.read_frame()
            if frame is None:
                raise RuntimeError("OpenCVCamera returned no frame during smoke test")
            observation = extract_laser_line(frame.image, color="blue", channel_order="rgb", min_points=20)
            summaries.append(
                {
                    "frame_index": frame_index,
                    "shape": list(frame.image.shape),
                    "stripe_points": len(observation.uv_points),
                    "metadata": frame.metadata,
                }
            )
    summary = {
        "video_path": str(VIDEO_PATH).replace("\\", "/"),
        "frames_checked": len(summaries),
        "first_frame": summaries[0],
    }
    (OUTPUT_DIR / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

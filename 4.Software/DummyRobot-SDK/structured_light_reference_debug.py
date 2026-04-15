import json
import os
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageSequence

sys.path.insert(0, os.path.abspath("src"))

from dummy_robot import (
    CameraIntrinsics,
    Plane3D,
    RigidTransform,
    draw_laser_overlay,
    extract_laser_line,
    fuse_point_cloud_frames,
    project_height_map,
    reconstruct_laser_points,
)


REFERENCE_GIF = Path("reference_assets/giulioz_input.gif")
OUTPUT_DIR = Path("test_outputs/reference_gif_debug")


def choose_color_channel(image_rgb: np.ndarray) -> str:
    candidates = {"red": 0, "green": 1, "blue": 2}
    best_name = "blue"
    best_score = -1.0
    for name, index in candidates.items():
        channel = image_rgb[..., index].astype(np.float32)
        score = float(np.percentile(channel, 99.7) - np.percentile(channel, 70.0))
        if score > best_score:
            best_name = name
            best_score = score
    return best_name


def main() -> None:
    if not REFERENCE_GIF.exists():
        raise FileNotFoundError(f"Reference asset not found: {REFERENCE_GIF}")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    overlays_dir = OUTPUT_DIR / "overlays"
    overlays_dir.mkdir(exist_ok=True)

    gif = Image.open(REFERENCE_GIF)
    frames = [np.array(frame.convert("RGB")) for frame in ImageSequence.Iterator(gif)]
    if not frames:
        raise RuntimeError("No frames found in reference GIF")

    selected_color = choose_color_channel(frames[0])
    intrinsics = CameraIntrinsics(fx=720.0, fy=720.0, cx=160.0, cy=120.0, image_width=320, image_height=240)
    laser_plane = Plane3D(normal_xyz=[0.10, 0.0, -0.995], offset=0.22)

    reconstructed_frames = []
    usable_frames = 0
    stripe_counts = []

    for index, frame in enumerate(frames):
        try:
            observation = extract_laser_line(
                image=frame,
                color=selected_color,
                channel_order="rgb",
                axis="columns",
                min_points=20,
            )
        except Exception:
            continue

        usable_frames += 1
        stripe_counts.append(len(observation.uv_points))
        overlay = draw_laser_overlay(frame, observation, color_rgb=(255, 0, 0))
        Image.fromarray(overlay).save(overlays_dir / f"frame_{index:03d}.png")

        transform = RigidTransform(
            rotation=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation_xyz=[index * 0.0015, 0.0, 0.0],
            from_frame="camera",
            to_frame="reference_scan",
        )
        reconstructed = reconstruct_laser_points(
            observation=type(observation)(
                timestamp=float(index),
                uv_points=observation.uv_points,
                intensities=observation.intensities,
                image_shape=observation.image_shape,
                metadata=observation.metadata,
            ),
            intrinsics=intrinsics,
            laser_plane_in_camera=laser_plane,
            transform=transform,
            output_frame="reference_scan",
        )
        reconstructed_frames.append(reconstructed)

    fused = fuse_point_cloud_frames(reconstructed_frames)
    height_map, height_meta = project_height_map(fused, grid_resolution=0.0025)
    np.savez_compressed(OUTPUT_DIR / "fused_pointcloud.npz", points_xyz=fused.astype(np.float32))
    np.savez_compressed(OUTPUT_DIR / "height_map.npz", height_map=height_map.astype(np.float32))

    summary = {
        "reference_asset": str(REFERENCE_GIF).replace("\\", "/"),
        "frame_count": len(frames),
        "usable_frames": usable_frames,
        "selected_color": selected_color,
        "avg_stripe_points": float(np.mean(stripe_counts)) if stripe_counts else 0.0,
        "fused_point_count": int(fused.shape[0]),
        "height_map_shape": list(height_map.shape),
        "height_map_metadata": height_meta,
        "note": "This run validates the extraction and reconstruction pipeline on public reference imagery. The reconstruction scale is illustrative because true calibration and robot poses are unavailable.",
    }
    (OUTPUT_DIR / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

from __future__ import annotations

from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

from .types import CameraIntrinsics, LaserLineObservation, Plane3D, ReconstructedPointCloudFrame, RigidTransform


def extract_laser_line(
    image: np.ndarray,
    threshold: Optional[float] = None,
    color: str = "blue",
    channel_order: str = "rgb",
    axis: str = "columns",
    min_points: int = 10,
) -> LaserLineObservation:
    """
    Extract the center line of a projected laser stripe from a single image.

    The default configuration assumes a blue line laser and RGB image layout.
    """

    image_array = np.asarray(image)
    if image_array.ndim not in (2, 3):
        raise ValueError("Image must be a 2D grayscale image or a 3D color image")

    response = _select_response_channel(image_array, color=color, channel_order=channel_order)
    working_threshold = float(threshold) if threshold is not None else _auto_threshold(response)
    uv_points: List[List[float]] = []
    intensities: List[float] = []

    if axis == "columns":
        for column in range(response.shape[1]):
            weights = response[:, column]
            mask = weights >= working_threshold
            if not np.any(mask):
                continue
            selected_rows = np.nonzero(mask)[0].astype(np.float64)
            selected_weights = weights[mask].astype(np.float64)
            center_row = float(np.sum(selected_rows * selected_weights) / np.sum(selected_weights))
            uv_points.append([float(column), center_row])
            intensities.append(float(np.max(selected_weights)))
    elif axis == "rows":
        for row in range(response.shape[0]):
            weights = response[row, :]
            mask = weights >= working_threshold
            if not np.any(mask):
                continue
            selected_cols = np.nonzero(mask)[0].astype(np.float64)
            selected_weights = weights[mask].astype(np.float64)
            center_col = float(np.sum(selected_cols * selected_weights) / np.sum(selected_weights))
            uv_points.append([center_col, float(row)])
            intensities.append(float(np.max(selected_weights)))
    else:
        raise ValueError("axis must be 'columns' or 'rows'")

    if len(uv_points) < min_points:
        raise ValueError(
            f"Laser extraction produced only {len(uv_points)} points; "
            f"threshold={working_threshold:.3f} may be too strict or the stripe is not visible"
        )

    return LaserLineObservation(
        timestamp=0.0,
        uv_points=uv_points,
        intensities=intensities,
        image_shape=list(image_array.shape),
        metadata={
            "threshold": working_threshold,
            "axis": axis,
            "color": color,
            "channel_order": channel_order,
        },
    )


def reconstruct_laser_points(
    observation: LaserLineObservation,
    intrinsics: CameraIntrinsics,
    laser_plane_in_camera: Plane3D,
    transform: Optional[RigidTransform] = None,
    output_frame: str = "camera",
) -> ReconstructedPointCloudFrame:
    rays = pixels_to_camera_rays(observation.uv_points, intrinsics)
    points = intersect_rays_with_plane(rays, laser_plane_in_camera)

    frame_id = output_frame
    metadata = {
        "intrinsics": intrinsics.as_dict(),
        "laser_plane_in_camera": laser_plane_in_camera.as_dict(),
    }
    if transform is not None:
        points = transform_points(points, transform)
        frame_id = transform.to_frame or output_frame
        metadata["transform"] = transform.as_dict()

    return ReconstructedPointCloudFrame(
        timestamp=observation.timestamp,
        points_xyz=points.astype(np.float64).tolist(),
        source_observation=observation,
        frame_id=frame_id,
        metadata=metadata,
    )


def pixels_to_camera_rays(uv_points: Sequence[Sequence[float]], intrinsics: CameraIntrinsics) -> np.ndarray:
    rays = []
    for u, v in uv_points:
        x = (float(u) - intrinsics.cx) / intrinsics.fx
        y = (float(v) - intrinsics.cy) / intrinsics.fy
        ray = np.asarray([x, y, 1.0], dtype=np.float64)
        rays.append(ray / np.linalg.norm(ray))
    if not rays:
        return np.empty((0, 3), dtype=np.float64)
    return np.vstack(rays)


def intersect_rays_with_plane(rays: np.ndarray, plane: Plane3D) -> np.ndarray:
    normal = plane.normal_vector()
    denominator = np.dot(rays, normal)
    valid = np.abs(denominator) > 1e-9
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float64)

    distances = np.full(rays.shape[0], np.nan, dtype=np.float64)
    distances[valid] = -plane.offset / denominator[valid]
    forward = np.logical_and(valid, distances > 0.0)
    if not np.any(forward):
        return np.empty((0, 3), dtype=np.float64)
    return rays[forward] * distances[forward][:, None]


def transform_points(points_xyz: np.ndarray, transform: RigidTransform) -> np.ndarray:
    points = np.asarray(points_xyz, dtype=np.float64)
    if points.size == 0:
        return points.reshape(0, 3)
    rotation = transform.rotation_matrix()
    translation = transform.translation_vector()
    return (rotation @ points.T).T + translation


def fuse_point_cloud_frames(frames: Iterable[ReconstructedPointCloudFrame]) -> np.ndarray:
    clouds = [np.asarray(frame.points_xyz, dtype=np.float64) for frame in frames if frame.points_xyz]
    if not clouds:
        return np.empty((0, 3), dtype=np.float64)
    return np.vstack(clouds)


def project_height_map(
    points_xyz: np.ndarray,
    grid_resolution: float = 0.002,
    fill_value: float = np.nan,
) -> Tuple[np.ndarray, dict]:
    points = np.asarray(points_xyz, dtype=np.float64)
    if points.size == 0:
        return np.empty((0, 0), dtype=np.float32), {"grid_resolution": grid_resolution, "origin_xy": [0.0, 0.0]}

    x_values = points[:, 0]
    y_values = points[:, 1]
    z_values = points[:, 2]
    x_min, x_max = float(np.min(x_values)), float(np.max(x_values))
    y_min, y_max = float(np.min(y_values)), float(np.max(y_values))

    width = int(np.floor((x_max - x_min) / grid_resolution)) + 1
    height = int(np.floor((y_max - y_min) / grid_resolution)) + 1
    grid = np.full((height, width), fill_value, dtype=np.float32)
    counts = np.zeros((height, width), dtype=np.int32)

    for x_value, y_value, z_value in zip(x_values, y_values, z_values):
        col = int(np.floor((x_value - x_min) / grid_resolution))
        row = int(np.floor((y_value - y_min) / grid_resolution))
        if np.isnan(grid[row, col]):
            grid[row, col] = np.float32(z_value)
        else:
            grid[row, col] = np.float32((grid[row, col] * counts[row, col] + z_value) / (counts[row, col] + 1))
        counts[row, col] += 1

    metadata = {
        "grid_resolution": grid_resolution,
        "origin_xy": [x_min, y_min],
        "shape": [height, width],
    }
    return grid, metadata


def fit_plane_from_points(points_xyz: np.ndarray) -> Plane3D:
    points = np.asarray(points_xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] < 3:
        raise ValueError("At least 3 3D points are required to fit a plane")
    centroid = np.mean(points, axis=0)
    centered = points - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1, :]
    normal /= np.linalg.norm(normal)
    offset = -float(np.dot(normal, centroid))
    return Plane3D(normal_xyz=normal.tolist(), offset=offset)


def draw_laser_overlay(
    image: np.ndarray,
    observation: LaserLineObservation,
    color_rgb: Sequence[int] = (255, 0, 0),
) -> np.ndarray:
    base = np.asarray(image).copy()
    if base.ndim == 2:
        base = np.stack([base, base, base], axis=-1)
    for u, v in observation.uv_points:
        col = int(round(u))
        row = int(round(v))
        if 0 <= row < base.shape[0] and 0 <= col < base.shape[1]:
            base[row, col, :3] = np.asarray(color_rgb[:3], dtype=base.dtype)
    return base


def generate_synthetic_laser_image(
    width: int = 640,
    height: int = 480,
    line_width_px: int = 4,
    color: str = "blue",
    slope: float = 0.2,
    intercept: Optional[float] = None,
    background: int = 18,
    laser_value: int = 255,
    noise_std: float = 4.0,
    channel_order: str = "rgb",
) -> np.ndarray:
    image = np.full((height, width, 3), background, dtype=np.float32)
    line_center = intercept if intercept is not None else height * 0.5
    half_width = max(1, int(line_width_px // 2))
    channel_index = _channel_index(color=color, channel_order=channel_order)

    for column in range(width):
        row_center = int(round(line_center + slope * (column - width * 0.5)))
        row_min = max(0, row_center - half_width)
        row_max = min(height, row_center + half_width + 1)
        image[row_min:row_max, column, channel_index] = laser_value

    if noise_std > 0:
        image += np.random.normal(0.0, noise_std, size=image.shape)

    return np.clip(image, 0, 255).astype(np.uint8)


def _select_response_channel(image: np.ndarray, color: str, channel_order: str) -> np.ndarray:
    if image.ndim == 2:
        return image.astype(np.float32)
    if image.shape[2] < 3:
        raise ValueError("Color images must contain at least 3 channels")
    channel_index = _channel_index(color=color, channel_order=channel_order)
    return image[..., channel_index].astype(np.float32)


def _channel_index(color: str, channel_order: str) -> int:
    order = channel_order.lower()
    if order not in ("rgb", "bgr"):
        raise ValueError("channel_order must be 'rgb' or 'bgr'")
    color_map = {"red": 0, "green": 1, "blue": 2}
    if color.lower() not in color_map:
        raise ValueError("color must be one of: red, green, blue")
    index = color_map[color.lower()]
    if order == "rgb":
        return index
    return {0: 2, 1: 1, 2: 0}[index]


def _auto_threshold(response: np.ndarray) -> float:
    high = float(np.percentile(response, 99.5))
    median = float(np.median(response))
    return max(20.0, median + (high - median) * 0.45)

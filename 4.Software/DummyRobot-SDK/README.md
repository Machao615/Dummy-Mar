# DummyRobot SDK

`DummyRobot SDK` is a Python package for controlling `DummyRobot` over the existing ASCII serial protocol and for building monocular structured-light scanning workflows around the robot.

The current scanning path is based on one concrete assumption:

- the plant stays still
- the robot carries the camera and the line-laser module together
- the robot moves the sensor head along a scan trajectory
- each scan pose produces one synchronized image frame
- the laser stripe is reconstructed into 3D points and fused across frames

## Requirements

- Python `>=3.7`
- `pyserial>=3.5`
- `numpy>=1.24`

Optional dependencies:

- `pyarrow>=15.0.0` for low-dimensional `LeRobot` export
- `Pillow>=10.0.0` for the public-reference structured-light debug script

## Installation

```bash
pip install -e .
```

Optional extras:

```bash
pip install -e .[lerobot]
pip install -e .[structured-light-debug]
```

## Packaging

```bash
python -m build
```

If `build` is not installed:

```bash
pip install build
python -m build
```

Windows shortcut:

```bash
build_package.bat
```

Artifacts are written to `dist/`.

## Feature Overview

### 1. Core robot control

The high-level `DummyRobot` client supports:

- `connect()`
- `disconnect()`
- `list_serial_ports()`
- `send_raw()`
- `enable()`
- `stop()`
- `disable()`
- `home()`
- `reset()`
- `calibrate()`
- `set_kp()`
- `set_ki()`
- `set_kd()`
- `reboot_motor()`
- `set_command_mode()`
- `get_joint_positions()`
- `get_tool_pose()`
- `move_to_pose()`
- `move_to_joints()`
- `move_joints()`
- `move_single_joint()`
- `send_gcode()`
- `get_observation()`
- `get_lerobot_observation()`
- `get_state_snapshot()`
- `execute_action()`

Wrapped firmware-side commands:

- `!START`
- `!STOP`
- `!DISABLE`
- `!HOME`
- `!RESET`
- `!CALIBRATION`
- `#GETJPOS`
- `#GETLPOS`
- `#SET_DCE_KP`
- `#SET_DCE_KI`
- `#SET_DCE_KD`
- `#REBOOT`
- `#CMDMODE`
- joint motion: `&...`
- sequential joint motion: `>...`
- Cartesian pose motion: `@...`

### 2. Camera and structured-light scanning

The package includes a dedicated image-based scanning layer:

- `CameraInterface`
- `CallbackCamera`
- `OpenCVCamera`
- `CameraPoseSynchronizer`
- `CameraFrame`
- `SyncedCameraFrame`
- `CameraIntrinsics`
- `Plane3D`
- `RigidTransform`
- `LaserLineObservation`
- `ReconstructedPointCloudFrame`
- `register_camera()`
- `capture_camera_frame()`
- `execute_structured_light_scan_task()`

Offline reconstruction helpers:

- `extract_laser_line()`
- `pixels_to_camera_rays()`
- `intersect_rays_with_plane()`
- `reconstruct_laser_points()`
- `transform_points()`
- `fuse_point_cloud_frames()`
- `fit_plane_from_points()`
- `project_height_map()`
- `draw_laser_overlay()`
- `generate_synthetic_laser_image()`

Structured-light task and storage:

- `StructuredLightWaypoint`
- `StructuredLightScanTask`
- `StructuredLightScanResult`
- `StructuredLightDatasetWriter`

### 3. Low-dimensional export

The package also keeps a low-dimensional `LeRobot` helper layer:

- `LeRobotDatasetWriter`
- `DummyRobotAdapter`

This layer intentionally excludes structured-light image and point-cloud data.

## How Monocular 3D Reconstruction Works

A single camera does not directly measure depth. The 3D constraint comes from the laser plane.

For each bright laser pixel:

1. The pixel `(u, v)` defines a camera ray once camera intrinsics are known.
2. The projected laser stripe belongs to a known 3D laser plane.
3. The intersection of the camera ray and the laser plane is the 3D point.

In short:

```text
pixel -> camera ray
laser stripe -> known laser plane
ray intersects plane = 3D point
```

Then each frame is transformed into a common robot coordinate system using robot pose and sensor extrinsics, and all frames are fused into one point cloud.

## Expected Scan Workflow

The intended hardware workflow is:

1. Rigidly mount the camera and line-laser module together on the end-effector.
2. Keep the plant stationary.
3. Move the robot along a smooth scan trajectory.
4. Capture one synchronized image frame per scan pose.
5. Extract the laser stripe from each image.
6. Reconstruct 3D points in the camera frame.
7. Transform those points into the robot base frame.
8. Fuse all frames into a single point cloud or height map.

## Basic Usage

### Connect and query state

```python
from dummy_robot import DummyRobot

with DummyRobot("COM3") as robot:
    robot.enable()

    joints = robot.get_joint_positions()
    print("Joint positions:", joints.as_list())

    pose = robot.get_tool_pose()
    print("Tool pose:", pose.as_list())

    robot.disable()
```

### Joint-space motion

```python
from dummy_robot import DummyRobot

with DummyRobot("COM3") as robot:
    robot.enable()
    robot.move_joints([0, -20, 110, 0, 0, 0], speed=10)
```

### Offline structured-light prototype

This workflow does not require the real robot or a real camera. It generates a synthetic laser image, extracts the stripe, reconstructs 3D points against a calibrated laser plane, and fuses multiple frames:

```python
from dummy_robot import (
    CameraIntrinsics,
    Plane3D,
    RigidTransform,
    extract_laser_line,
    fuse_point_cloud_frames,
    generate_synthetic_laser_image,
    project_height_map,
    reconstruct_laser_points,
)

intrinsics = CameraIntrinsics(fx=820.0, fy=820.0, cx=320.0, cy=240.0)
laser_plane = Plane3D(normal_xyz=[0.15, 0.0, -0.99], offset=0.18)

frames = []
for index in range(3):
    image = generate_synthetic_laser_image(intercept=180 + index * 18, slope=0.08)
    observation = extract_laser_line(image=image, color="blue", channel_order="rgb")
    transform = RigidTransform(
        rotation=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        translation_xyz=[index * 0.015, 0.0, 0.0],
        from_frame="camera",
        to_frame="robot_base",
    )
    frames.append(
        reconstruct_laser_points(
            observation=observation,
            intrinsics=intrinsics,
            laser_plane_in_camera=laser_plane,
            transform=transform,
            output_frame="robot_base",
        )
    )

fused = fuse_point_cloud_frames(frames)
height_map, metadata = project_height_map(fused, grid_resolution=0.003)
print(fused.shape, height_map.shape, metadata)
```

An end-to-end offline example is available in `structured_light_offline_demo.py`.

## Reference Materials and Debug Assets

Local resources prepared for follow-up debugging:

- `STRUCTURED_LIGHT_RESOURCES_CN.md`
- `reference_assets/giulioz_input.gif`
- `reference_assets/giulioz_reference_README.md`
- `structured_light_reference_debug.py`

To run the public-reference debug pipeline:

```bash
python structured_light_reference_debug.py
```

This script consumes the downloaded public GIF reference, extracts laser stripes frame by frame, writes overlay images, and exports fused debug artifacts into `test_outputs/reference_gif_debug`.

## Real Camera Integration

The SDK includes `OpenCVCamera` so you can connect a real USB camera or a video file through OpenCV.

Example:

```python
from dummy_robot import DummyRobot, OpenCVCamera

with DummyRobot("COM3") as robot:
    with OpenCVCamera(name="scan_camera", source=0) as camera:
        robot.register_camera(camera)
        synced = robot.capture_camera_frame("scan_camera")
        print(synced.frame.metadata)
```

To load camera intrinsics from JSON:

```python
from dummy_robot import OpenCVCamera

camera = OpenCVCamera.from_json(
    name="scan_camera",
    source=0,
    intrinsics_path="camera_intrinsics.json",
    undistort=True,
)
```

Expected JSON format:

```json
{
  "fx": 820.0,
  "fy": 820.0,
  "cx": 320.0,
  "cy": 240.0,
  "dist_coeffs": [0.01, -0.02, 0.0, 0.0, 0.0],
  "image_width": 640,
  "image_height": 480
}
```

Local smoke test for the OpenCV camera path:

```bash
python opencv_camera_smoke_test.py
```

## Package Structure

Main modules:

- `client.py`
- `transport.py`
- `types.py`
- `exceptions.py`
- `sensors.py`
- `camera.py`
- `structured_light.py`
- `structured_light_io.py`
- `structured_light_tasks.py`
- `dataset.py`
- `lerobot_adapter.py`

## Recommended Entry Points

For robot control:

- `DummyRobot`
- `JointPositions`
- `Pose6D`
- `RobotAction`

For structured-light scanning:

- `CameraIntrinsics`
- `Plane3D`
- `RigidTransform`
- `extract_laser_line()`
- `reconstruct_laser_points()`
- `StructuredLightScanTask`
- `StructuredLightDatasetWriter`

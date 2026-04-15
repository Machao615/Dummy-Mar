# DummyRobot SDK 中文说明

`DummyRobot SDK` 是一个面向 `DummyRobot` 的 Python 开发包，用于通过现有 ASCII 串口协议控制机械臂，并围绕机械臂搭建基于单目相机与一字线激光的结构光扫描流程。

当前扫描主线已经统一为：

- 植物固定不动
- 机械臂末端同时安装相机和一字线激光模组
- 机械臂带着整套传感头按轨迹匀速或分步移动
- 每个位姿采集一帧图像
- 从图像中提取激光线并恢复 3D 点
- 多帧点云融合成完整表面

## 1. 运行要求

- Python `>=3.7`
- `pyserial>=3.5`
- `numpy>=1.24`

可选依赖：

- `pyarrow>=15.0.0`
  - 用于低维 `LeRobot` 导出
- `opencv-python>=4.8.0`
  - 用于 `OpenCVCamera`
- `Pillow>=10.0.0`
  - 用于公开参考 GIF 调试脚本

## 2. 安装

```bash
pip install -e .
```

如需使用可选能力：

```bash
pip install -e .[lerobot]
pip install -e .[structured-light-debug]
```

## 3. 打包

```bash
python -m build
```

如果当前环境没有安装 `build`：

```bash
pip install build
python -m build
```

Windows 下也可以直接执行：

```bash
build_package.bat
```

打包产物默认输出到 `dist/`。

## 4. 当前功能概览

### 4.1 机械臂串口控制

`DummyRobot` 当前支持：

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
- `get_state_snapshot()`
- `execute_action()`

### 4.2 结构光扫描接口

当前已经内置一套面向“单目相机 + 一字线激光”的结构光扫描层：

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

### 4.3 离线算法工具

当前已经实现的核心算法接口：

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

这些接口可以在没有真实硬件时先完成：

- 激光线提取验证
- 单帧三维重建验证
- 多帧点云融合验证
- 高度图生成验证
- 调试数据写出验证

### 4.4 结构光任务与数据写出

当前已经加入结构光任务层：

- `StructuredLightWaypoint`
- `StructuredLightScanTask`
- `StructuredLightScanResult`
- `StructuredLightDatasetWriter`

它们适合用于：

- 机械臂按轨迹移动并同步采图
- 每帧图像自动提线与重建
- 保存图像、位姿和点云结果

### 4.5 真实相机接入

当前已经加入 `OpenCVCamera`，可以直接接：

- USB 相机设备号，例如 `0`
- 视频文件路径

支持能力：

- 自动通过 OpenCV 读取图像
- 可选图像去畸变
- 可选 `BGR -> RGB` 转换
- 可从 JSON 文件加载相机内参

推荐接入方式：

1. 先标定相机，得到 `fx/fy/cx/cy/dist_coeffs`
2. 写成一个 JSON 文件
3. 用 `OpenCVCamera.from_json(...)` 创建相机对象
4. 注册到 `DummyRobot`
5. 直接走 `capture_camera_frame()` 或 `StructuredLightScanTask`

### 4.6 低维导出层

SDK 仍保留：

- `LeRobotDatasetWriter`
- `DummyRobotAdapter`

这部分只处理机械臂低维状态和动作，不保存结构光图像或点云。

## 5. 单目相机为什么可以恢复 3D 点云

关键点在于：单目相机本身不直接测深度，但激光平面提供了额外的空间约束。

对图像中的每一个激光亮点：

1. 像素点 `(u, v)` 经过相机内参换算后，可以得到一条从相机光心发出的空间射线。
2. 这一点又必然落在“激光平面”上，因为它来自一字线激光投射。
3. 这条射线与激光平面的交点，就是该像素对应的真实 3D 点。

也就是：

```text
像素点 -> 相机射线
激光线 -> 已知激光平面
射线与平面求交 = 3D 点
```

因此你需要先知道三样东西：

- 相机内参
- 激光平面参数
- 相机/末端/机器人基坐标系之间的变换关系

多帧情况下，再把每一帧的点云通过机械臂位姿变换到统一坐标系，就能得到完整的植物表面点云。

## 6. 预期扫描流程

建议按以下链路实现：

1. 将相机和一字线激光模组刚性固定在机械臂末端。
2. 保持植物不动。
3. 机械臂按照扫描轨迹移动传感头。
4. 每个扫描位姿采集一帧图像。
5. 从图像中提取激光中心线。
6. 将激光中心线恢复为相机坐标系下的 3D 点。
7. 通过外参与机械臂位姿变换到机器人基坐标系。
8. 融合所有帧，得到点云或高度图。

## 7. 基础使用示例

### 7.1 连接并读取机械臂状态

```python
from dummy_robot import DummyRobot

with DummyRobot("COM3") as robot:
    robot.enable()

    joints = robot.get_joint_positions()
    print("关节角：", joints.as_list())

    pose = robot.get_tool_pose()
    print("末端位姿：", pose.as_list())

    robot.disable()
```

### 7.2 离线结构光原型

下面的示例不依赖真实机械臂和相机。它会生成合成激光图像、提取激光线并重建 3D 点云：

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

完整离线示例见：

- `structured_light_offline_demo.py`
- `structured_light_reference_debug.py`

### 7.3 真实相机接入示例

```python
from dummy_robot import DummyRobot, OpenCVCamera

with DummyRobot("COM3") as robot:
    with OpenCVCamera(name="scan_camera", source=0) as camera:
        robot.register_camera(camera)
        synced = robot.capture_camera_frame("scan_camera")
        print(synced.frame.metadata)
```

如果已经有标定参数，可以直接从 JSON 加载：

```python
from dummy_robot import OpenCVCamera

camera = OpenCVCamera.from_json(
    name="scan_camera",
    source=0,
    intrinsics_path="camera_intrinsics.json",
    undistort=True,
)
```

推荐的 JSON 格式：

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

本地 OpenCV 相机链路验证脚本：

- `opencv_camera_smoke_test.py`

## 8. 当前包结构

当前主要模块包括：

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

## 9. 推荐入口

如果你的当前目标是“植物表面结构光扫描”，建议优先使用：

- `DummyRobot`
- `OpenCVCamera`
- `CameraIntrinsics`
- `Plane3D`
- `RigidTransform`
- `extract_laser_line()`
- `reconstruct_laser_points()`
- `StructuredLightScanTask`
- `StructuredLightDatasetWriter`

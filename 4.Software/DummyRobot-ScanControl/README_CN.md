# DummyRobot Scan Control

这个小项目只做一件事：**通过 `DummyRobot-SDK` 控制 `DummyRobot` 做均匀扫描运动**。

当前目标场景是：

- 机械臂末端搭载激光模组
- 植物固定不动
- 机械臂沿预设关节轨迹缓慢、均匀移动
- 每一步暂停一小段时间，给后续相机采图或人工观察留出时间

## 1. 适用方式

当前实现的是“单关节均匀步进扫描”：

- 先把机械臂移动到一个固定初始姿态
- 指定某一个关节
- 每一步对该关节增加或减少固定角度
- 每一步之间暂停固定时间
- 扫描结束后回到初始姿态

这正对应你当前给出的控制方式。

## 2. 安装

在项目目录执行：

```bash
pip install -e ../DummyRobot-SDK
pip install -e .
```

## 3. 最简单用法

示例：让第 3 关节从 `150` 度开始，每步减小 `0.1` 度，共执行 `200` 步。

```bash
dummy-robot-uniform-scan ^
  --port COM3 ^
  --joint-index 3 ^
  --step-angle -0.1 ^
  --steps 200 ^
  --speed 50 ^
  --pause 0.5 ^
  --initial-joints 0,-60,150,0,0,0
```

Linux/macOS 下同理，只需要把串口改成例如 `/dev/ttyACM0`。

## 4. 配置文件方式

你也可以直接改配置文件：

- `configs/joint3_uniform_scan.example.json`

运行：

```bash
dummy-robot-uniform-scan --config configs/joint3_uniform_scan.example.json
```

主要字段说明：

- `port`
  - 串口名，例如 `COM3` 或 `/dev/ttyACM0`
- `initial_joints`
  - 初始 6 关节角度
- `joint_index`
  - 要扫描的关节编号，范围 `1~6`
- `step_angle_deg`
  - 每一步的角度变化量，负数表示减小，正数表示增大
- `total_steps`
  - 总步数
- `speed`
  - 指令中的速度参数
- `pause_time_s`
  - 每一步之间的等待时间
- `return_to_initial`
  - 扫描结束后是否回到初始姿态
- `log_path`
  - 每一步日志的输出文件

## 5. 安全说明

实际联调前建议：

1. 先用较小的步数和较大的暂停时间
2. 先确认初始姿态不会撞到植物、夹具或桌面
3. 第一轮先不开激光和相机，只看机械臂运动是否平稳
4. 确认扫描方向是否正确
5. 如需急停，直接中断进程

当前脚本捕获 `Ctrl+C` 后会进入清理流程，并默认尝试回到初始姿态。

## 6. 干跑模式

如果你现在还没接机械臂，可以先验证命令生成流程：

```bash
dummy-robot-uniform-scan ^
  --port COM3 ^
  --joint-index 3 ^
  --step-angle -0.1 ^
  --steps 5 ^
  --speed 50 ^
  --pause 0.1 ^
  --initial-joints 0,-60,150,0,0,0 ^
  --dry-run
```

这不会打开串口，但会完整走一遍配置和流程。

## 7. 当前项目结构

- `src/dummy_robot_scan_control/controller.py`
  - 串口控制和扫描执行逻辑
- `src/dummy_robot_scan_control/cli.py`
  - 命令行入口
- `configs/joint3_uniform_scan.example.json`
  - 示例配置

## 8. 与你给出的原始示例的关系

你给的示例是直接使用 `pyserial` 手写串口指令。

当前这个项目保留了同样的控制思路，但底层已经改成调用 `DummyRobot-SDK`：

- 初始位使用 SDK 的 `move_joints()`
- 扫描步进使用 SDK 的 `move_single_joint()`
- 启停使用 SDK 的 `enable()` / `disable()`
- 暂停节奏仍然使用 `time.sleep()`

这样做的好处是：

- 后续如果 SDK 对协议层有修改，这个项目不用重写串口细节
- 扫描控制逻辑和 SDK 主线保持一致
- 后面接相机和结构光任务时更容易并到同一套工程里

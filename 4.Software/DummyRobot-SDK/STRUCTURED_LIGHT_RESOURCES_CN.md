# 结构光扫描资料与后续调试说明

这份文档不是理论课笔记，而是给后续联调和软件开发直接用的工作说明。

## 1. 当前已经帮你准备好的本地资源

### 1.1 已下载的公开参考

当前工作区已经拉取并使用过以下公开参考：

- Python 结构光扫描参考仓：
  - [giulioz/laser-scanning](https://github.com/giulioz/laser-scanning)
- ROS 激光条纹重建参考仓：
  - [srv/laser_stripe_reconstruction](https://github.com/srv/laser_stripe_reconstruction)

其中更适合你当前 SDK 路线的是 `giulioz/laser-scanning`，因为它本身就是：

- Python
- OpenCV + NumPy
- 线激光结构光
- 相机标定 + 条纹提取 + 3D 重建

### 1.2 已放进 SDK 的本地参考资产

已复制到 SDK 内部：

- `reference_assets/giulioz_input.gif`
- `reference_assets/giulioz_reference_README.md`

这个 `gif` 来自公开参考仓的输入示例，可作为你后续离线调试的固定参考输入。

## 2. 当前已经跑通过的离线调试入口

### 2.1 合成数据调试

脚本：

- `structured_light_offline_demo.py`

用途：

- 生成合成激光图像
- 提取激光线
- 单帧三维重建
- 多帧点云融合
- 高度图生成

### 2.2 公开参考数据调试

脚本：

- `structured_light_reference_debug.py`

用途：

- 读取 `reference_assets/giulioz_input.gif`
- 自动选择较合适的颜色通道
- 对每帧执行激光线提取
- 生成叠加调试图
- 执行示意性三维重建
- 生成融合点云和高度图

输出目录：

- `test_outputs/reference_gif_debug`

主要输出包括：

- `summary.json`
- `fused_pointcloud.npz`
- `height_map.npz`
- `overlays/frame_*.png`

注意：

- 这套参考输入没有你的真实相机标定和真实机械臂位姿
- 所以这里的 3D 重建只用于验证软件链路，不代表真实尺度精度

## 3. 你后续真正要看的资料

### 3.1 相机模型和标定

- OpenCV 官方文档：
  - [OpenCV calib3d](https://docs.opencv.org/master/d9/d0c/group__calib3d.html)

这个部分要解决的是：

- 相机内参怎么表示
- 像素点如何变成相机射线
- 畸变如何处理

### 3.2 线结构光标定

- 论文：
  - [Calibration Method for Line-Structured Light 3D Measurement](https://www.mdpi.com/2304-6732/9/4/218)

这个部分要解决的是：

- 激光平面如何标定
- 为什么单目相机也能恢复 3D
- 射线和平面如何求交

### 3.3 激光条纹工程实现

- 参考仓：
  - [giulioz/laser-scanning](https://github.com/giulioz/laser-scanning)
  - [srv/laser_stripe_reconstruction](https://github.com/srv/laser_stripe_reconstruction)

这个部分要解决的是：

- 条纹阈值如何选
- 图像中哪些点拿来做重建
- 如何把多帧点云聚合

### 3.4 植物点云后处理

如果后续你要练点云处理，而不是练条纹提取，可以参考：

- [Pheno4D](https://www.ipb.uni-bonn.de/data/pheno4d/index.html)
- [Crops3D](https://huggingface.co/datasets/Voxel51/crops3d)

注意：

- 这两类更适合做点云处理、分割、可视化
- 不适合调你现在的“图像提线 + 激光平面求交”核心链路

## 4. 当前这套 SDK 里你最该继续调的部分

优先顺序建议如下：

1. `extract_laser_line()`
   - 调阈值策略
   - 调颜色通道选择
   - 看 `overlays` 是否稳定

2. `reconstruct_laser_points()`
   - 调假定内参和激光平面
   - 确认点云形状和尺度变化是否符合预期

3. `fuse_point_cloud_frames()`
   - 确认多帧拼接逻辑没问题

4. `project_height_map()`
   - 调网格分辨率
   - 看高度图是否平滑、是否有明显空洞

## 5. 等你回到设备旁边后，最先补的真实数据

回去后不要立刻扫植物，先补这三类真实数据：

1. 相机标定图像
   - 棋盘格多视角图像

2. 静态激光条纹图像
   - 植物不动，机械臂不动，只拍激光线

3. 简单平面或规则物体扫描数据
   - 先验证重建是否正确

这样可以把问题拆开：

- 图像提线问题
- 标定问题
- 位姿同步问题
- 点云融合问题

## 6. 当前建议的命令

在 SDK 根目录执行：

```bash
python structured_light_offline_demo.py
```

以及：

```bash
python structured_light_reference_debug.py
```

如果这两条都稳定，你后续接真实相机时，主要改的是“输入源”和“标定参数”，不是整条处理链。

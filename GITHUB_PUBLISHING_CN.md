# GitHub 发布清单

本文档用于整理 `Dummy-Mar` 在发布到 GitHub 前的仓库边界、建议保留内容和最终检查项。

## 建议公开的内容

- `1.Hardware/`
  - 原理图、PCB 源文件、Gerber、制造输出
- `2.Model/`
  - STEP、STL、工程图、结构修订模型
- `3.Firmware/`
  - 主控固件、驱动固件、相关嵌入式源码
- `4.Software/DummyRobot-SDK/`
  - Python SDK、结构光采图与离线算法原型
- `4.Software/DummyRobot-ScanControl/`
  - 基于 SDK 的扫描运动控制工程
- `4.Software/DummyRobot-WebConsole/`
  - Web 上位机源码
- `4.Software/CLI-Tool/`
  - 如确认可公开，可保留源码部分
- `4.Software/Simple-CLI/`
  - 简单串口工具
- `5.Misc/`
  - 仅保留确认可以公开的工程辅助资料

## 建议不要直接提交到 GitHub 的内容

- `6.Thesis/`
  - 论文、任务书、开题、中期、答辩材料
- `_tools/`
  - 本地工具链、FreeCAD 运行目录、下载缓存
- `4.Software/DummyStudio/`
  - 编译后的上位机发布产物
- `4.Software/*.zip`
  - 本地打包结果和备份压缩包
- `4.Software/_temp_laser_scanning_ref/`
- `4.Software/_temp_laser_stripe_ref/`
  - 临时参考仓
- `4.Software/DummyRobot-SDK/test_outputs/`
- `4.Software/DummyRobot-ScanControl/outputs/`
  - 本地测试输出、扫描日志、调试结果
- IDE 和缓存目录
  - `.idea/`
  - `.vscode/`
  - `__pycache__/`

## 重点提醒

### 1. 不要把“源码仓”和“发布包仓”混在一起

当前目录中存在两类明显不适合源码仓的内容：

- 已编译应用
- 本地工具链
- 压缩归档
- 驱动安装工具

这些内容更适合：

- 放到 GitHub Releases
- 放到网盘或对象存储
- 在 README 中提供下载说明

### 2. `5.Misc/` 需要人工二次筛选

`5.Misc/` 里可能包含：

- 芯片说明书
- 第三方 PDF
- 安装包
- 供应商资料

这些文件不一定适合再分发。上传前建议逐项确认版权和公开性。

### 3. `CLI-Tool/` 需要确认第三方代码来源

`4.Software/CLI-Tool/` 内含：

- `fibre`
- `ref_tool`
- 驱动安装工具

发布前建议确认：

- 哪些是你自己的代码
- 哪些是第三方依赖副本
- 是否需要改为 README 中说明安装方式，而不是把副本直接提交进仓库

## 推荐的仓库主页结构

建议 GitHub 首页重点展示：

1. 项目简介
2. 机械臂整体结构
3. 软件组成
4. 结构光扫描链路
5. 快速开始
6. 目录说明
7. 非公开内容说明

## 发布前最终检查

按顺序执行：

1. 确认 `6.Thesis/` 未被纳入版本管理
2. 确认 `_tools/` 未被纳入版本管理
3. 确认 `DummyStudio/` 未被纳入版本管理
4. 确认 `4.Software/*.zip` 未被纳入版本管理
5. 确认本地输出目录、截图、缓存未被纳入版本管理
6. 检查 `5.Misc/` 中是否有不适合公开分发的第三方文件
7. 检查根目录 `README.md` 是否已经反映当前公开边界
8. 再执行一次仓库体积检查，避免大文件直接进入首个提交

## 如果要进一步收缩仓库体积

优先考虑这些目录是否改为外部下载：

- `2.Model/` 中的大型导出文件
- `5.Misc/` 中的供应商 PDF 和压缩包
- 编译后的上位机发布产物
- 本地工具链目录

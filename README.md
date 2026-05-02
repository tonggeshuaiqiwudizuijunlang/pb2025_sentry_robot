<div align="center">

# pb2025_sentry_robot

**Full-Stack ROS 2 Algorithm Workspace for RoboMaster 2025 Sentry Robot**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)](https://ubuntu.com/)

[English](#english) | [中文](#中文)

</div>

---

<a name="english"></a>

## 🤖 Overview

**pb2025_sentry_robot** is a complete, open-source ROS 2 algorithm workspace for the RoboMaster 2025 Sentry robot. It integrates the full autonomous-robot software stack — from low-level hardware communication to high-level behavior decision-making — enabling the Sentry to operate fully autonomously during competition.

### Key Features

| Module | Technology | Description |
|--------|-----------|-------------|
| Vision & Auto-Aim | OpenVINO · EKF · Hikvision SDK | Armor detection, tracking & ballistic compensation |
| Navigation & Mapping | Point-LIO · Nav2 | 3-D LiDAR SLAM and autonomous patrol |
| Decision & Behavior | BehaviorTree.CPP v4 | BT-based shoot control & status management |
| Hardware Comm. | serial / CAN | MCU bridge for chassis & gimbal |
| Simulation | Gazebo Harmonic | Physics simulation & sensor emulation |

---

## 📦 Repository Structure

```
pb2025_sentry_robot/          # ROS 2 workspace root
├── src/
│   ├── rm_vision/            # Vision & Auto-Aim (FYT/rm_vision)
│   │   ├── hik_camera/       # Hikvision camera driver (MV-CS series)
│   │   ├── armor_detector/   # OpenVINO-based armor plate detector
│   │   ├── armor_tracker/    # Adaptive EKF target tracker
│   │   └── ballistic_solver/ # Gravity-compensated trajectory solver
│   ├── pb2025_sentry_nav/    # Navigation & Mapping
│   │   ├── point_lio_ros2/   # Point-LIO 3-D LiDAR odometry
│   │   ├── slam_toolbox/     # 2-D SLAM fallback
│   │   └── nav2_bringup/     # Nav2 costmaps & planner config
│   ├── pb2025_nav2_patrol/   # Autonomous patrol waypoint logic
│   ├── pb2025_sentry_behavior/ # Behavior Tree decision engine
│   │   ├── bt_nodes/         # Custom BT action & condition nodes
│   │   └── trees/            # XML BT definition files
│   ├── standard_robot_pp_ros2/ # Serial/CAN hardware bridge
│   │   ├── protocol/         # Custom robot communication protocol
│   │   └── drivers/          # UART / CAN driver wrappers
│   └── rmu_gazebo_simulator/ # Gazebo simulation environment
│       ├── worlds/           # Competition field models
│       ├── models/           # Robot URDF / SDF descriptions
│       └── launch/           # Simulation launch files
├── .gitignore
├── LICENSE
└── README.md
```

---

## 🔧 Prerequisites

| Dependency | Version | Notes |
|------------|---------|-------|
| Ubuntu | 22.04 LTS | Recommended platform |
| ROS 2 | Humble Hawksbill | `ros-humble-desktop` |
| OpenVINO Toolkit | ≥ 2022.3 | Intel inference runtime |
| Hikvision MVS SDK | ≥ 4.3 | `libMVSDK.so` from MV-CC/CS series |
| Nav2 | Humble | `ros-humble-navigation2` |
| BehaviorTree.CPP | v4 | `ros-humble-behaviortree-cpp` |
| Gazebo | Harmonic | `gz-harmonic` |
| PCL | ≥ 1.12 | Point cloud library |
| Eigen3 | ≥ 3.4 | Linear algebra |

---

## 🚀 Getting Started

### 1 · Clone the Workspace

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/tonggeshuaiqiwudizuijunlang/pb2025_sentry_robot.git
```

### 2 · Install ROS 2 Dependencies

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3 · Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 4 · Launch (Hardware)

```bash
# Full sentry bringup (camera + LiDAR + nav + behavior)
ros2 launch pb2025_sentry_behavior sentry_bringup.launch.py
```

### 5 · Launch (Simulation)

```bash
# Start Gazebo environment
ros2 launch rmu_gazebo_simulator sim_bringup.launch.py
# In a second terminal – start navigation + behavior
ros2 launch pb2025_sentry_behavior sentry_sim_bringup.launch.py
```

---

## 🗺️ Module Details

### 1 · Vision & Auto-Aim (`rm_vision`)

The vision pipeline is built on the [FYT/rm_vision](https://github.com/FYT-PB2025-dong/rm_vision) framework:

- **Camera Driver** (`hik_camera`): Supports Hikvision MV-CS series via the official MVS SDK. Publishes `/camera/image_raw` and calibration info.
- **Armor Detector** (`armor_detector`): Lightweight CNN (YOLOX-tiny) running on Intel OpenVINO with async inference, providing < 5 ms detection latency.
- **Armor Tracker** (`armor_tracker`): Adaptive EKF that estimates target armor pose and velocity in 3-D; handles target switching and spinning armor identification.
- **Ballistic Solver** (`ballistic_solver`): Iterative gravity-compensated trajectory solver that outputs a corrected pitch/yaw command for the gimbal.

### 2 · Navigation & Mapping (`pb2025_sentry_nav`)

- **Point-LIO**: Direct-integration 3-D LiDAR odometry for solid-state / spinning LiDARs without scan matching, providing robust pose at ≥ 100 Hz.
- **Nav2 Stack**: Costmap2D, AMCL localisation, Smac Planner, and DWB local planner configured for the sentry chassis dynamics.

### 3 · Autonomous Patrol (`pb2025_nav2_patrol`)

Implements a configurable waypoint-based patrol loop using Nav2's `NavigateThroughPoses` action. Patrol strategies (random, ordered, threat-reactive) are selectable at runtime.

### 4 · Decision & Behavior (`pb2025_sentry_behavior`)

A BehaviorTree.CPP v4-based decision engine with custom nodes:

| Node | Type | Description |
|------|------|-------------|
| `CheckRobotAlive` | Condition | Query HP from hardware bridge |
| `CheckEnemyDetected` | Condition | Query vision pipeline |
| `AutoAimShoot` | Action | Enable gimbal auto-aim & shooter |
| `PatrolNavigate` | Action | Send patrol goal to Nav2 |
| `EmergencyReturn` | Action | Navigate to safe zone |

### 5 · Hardware Communication (`standard_robot_pp_ros2`)

Serial/CAN bridge between ROS 2 and the STM32-based chassis/gimbal MCU. Implements the competition standard protocol:
- Receives: robot HP, game status, enemy position (from referee system).
- Sends: gimbal angle commands, shooter enable, chassis velocity override.

### 6 · Simulation (`rmu_gazebo_simulator`)

Full physics simulation of the 2025 RoboMaster competition field including:
- Accurate sentry robot URDF with differential/wheeled chassis.
- Gazebo plugins for camera, IMU, and LiDAR sensor emulation.
- Referee system HP and game status mock nodes.

---

## 🙏 Acknowledgments / Based On

This project stands on the shoulders of the following outstanding open-source contributions. We are deeply grateful to each team and author:

| Project | Contributor | Usage in This Repo |
|---------|------------|-------------------|
| [rmoss / RoboMaster OSS](https://github.com/robomaster-oss) | RoboMaster OSS Community | Core ROS 2 interfaces, message definitions, and toolchain conventions |
| [TUP-Infantry-Vision](https://github.com/tup-robomaster) | Tsinghua University (TUP) Infantry Team | YOLOX armor detection model weights and training pipeline |
| [OpenVINO Async Inference](https://github.com/ev3rm0re/Vision) | Sichuan University OpenVINO Vision Team | Asynchronous OpenVINO inference wrapper for real-time detector |
| [rm.cv.fans / Adaptive EKF](https://github.com/julyfun/rm.cv.fans) | Shanghai Jiao Tong University (julyfun) | Adaptive Extended Kalman Filter design for armor target tracking |
| [Point-LIO](https://github.com/hku-mars/Point-LIO) | HKU MARS Lab | Point-wise LiDAR inertial odometry for 3-D mapping |
| [Nav2](https://github.com/ros-navigation/navigation2) | Open Robotics / Nav2 Community | Navigation2 behavior trees, planners, and costmaps |

> If we have inadvertently missed an attribution, please open an issue and we will correct it immediately.

---

## 📜 License

This project is released under the [MIT License](LICENSE). Third-party components retain their respective licenses — please refer to each submodule for details.

---

## 🤝 Contributing

Contributions, issues, and feature requests are welcome! Please read [CONTRIBUTING.md](CONTRIBUTING.md) before submitting a pull request.

1. Fork the repository.
2. Create a feature branch: `git checkout -b feat/your-feature`.
3. Commit your changes with a descriptive message.
4. Open a pull request against `main`.

---

<a name="中文"></a>

## 🤖 项目概述

**pb2025_sentry_robot** 是 RoboMaster 2025 哨兵机器人的完整开源 ROS 2 算法工作空间，集成了从底层硬件通信到上层行为决策的全栈自主机器人软件系统，使哨兵能够在比赛中全自主运行。

### 核心特性

| 模块 | 技术栈 | 描述 |
|------|-------|------|
| 视觉与自瞄 | OpenVINO · EKF · 海康SDK | 装甲板检测、跟踪与弹道补偿 |
| 导航与建图 | Point-LIO · Nav2 | 三维激光雷达 SLAM 与自主巡逻 |
| 决策与行为 | BehaviorTree.CPP v4 | 基于行为树的射击控制与状态管理 |
| 硬件通信 | serial / CAN | 底盘与云台 MCU 通信桥接 |
| 仿真环境 | Gazebo Harmonic | 物理仿真与传感器模拟 |

---

## 📦 仓库结构

```
pb2025_sentry_robot/          # ROS 2 工作空间根目录
├── src/
│   ├── rm_vision/            # 视觉与自瞄 (FYT/rm_vision)
│   │   ├── hik_camera/       # 海康相机驱动 (MV-CS 系列)
│   │   ├── armor_detector/   # 基于 OpenVINO 的装甲板检测器
│   │   ├── armor_tracker/    # 自适应 EKF 目标跟踪器
│   │   └── ballistic_solver/ # 重力补偿弹道解算器
│   ├── pb2025_sentry_nav/    # 导航与建图
│   │   ├── point_lio_ros2/   # Point-LIO 三维激光雷达里程计
│   │   ├── slam_toolbox/     # 二维 SLAM 备用方案
│   │   └── nav2_bringup/     # Nav2 代价地图与规划器配置
│   ├── pb2025_nav2_patrol/   # 自主巡逻路点逻辑
│   ├── pb2025_sentry_behavior/ # 行为树决策引擎
│   │   ├── bt_nodes/         # 自定义 BT 动作节点与条件节点
│   │   └── trees/            # XML 行为树定义文件
│   ├── standard_robot_pp_ros2/ # 串口/CAN 硬件桥接
│   │   ├── protocol/         # 自定义机器人通信协议
│   │   └── drivers/          # UART / CAN 驱动封装
│   └── rmu_gazebo_simulator/ # Gazebo 仿真环境
│       ├── worlds/           # 比赛场地模型
│       ├── models/           # 机器人 URDF / SDF 描述
│       └── launch/           # 仿真启动文件
├── .gitignore
├── LICENSE
└── README.md
```

---

## 🔧 依赖环境

| 依赖项 | 版本 | 说明 |
|--------|------|------|
| Ubuntu | 22.04 LTS | 推荐系统 |
| ROS 2 | Humble Hawksbill | `ros-humble-desktop` |
| OpenVINO Toolkit | ≥ 2022.3 | Intel 推理运行时 |
| 海康 MVS SDK | ≥ 4.3 | `libMVSDK.so`，适用于 MV-CC/CS 系列 |
| Nav2 | Humble | `ros-humble-navigation2` |
| BehaviorTree.CPP | v4 | `ros-humble-behaviortree-cpp` |
| Gazebo | Harmonic | `gz-harmonic` |
| PCL | ≥ 1.12 | 点云库 |
| Eigen3 | ≥ 3.4 | 线性代数库 |

---

## 🚀 快速开始

### 1 · 克隆工作空间

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/tonggeshuaiqiwudizuijunlang/pb2025_sentry_robot.git
```

### 2 · 安装 ROS 2 依赖

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3 · 编译

```bash
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 4 · 启动（实车）

```bash
# 完整哨兵启动（相机 + 激光雷达 + 导航 + 行为）
ros2 launch pb2025_sentry_behavior sentry_bringup.launch.py
```

### 5 · 启动（仿真）

```bash
# 启动 Gazebo 环境
ros2 launch rmu_gazebo_simulator sim_bringup.launch.py
# 在另一个终端中启动导航与行为
ros2 launch pb2025_sentry_behavior sentry_sim_bringup.launch.py
```

---

## 🗺️ 模块详情

### 1 · 视觉与自瞄 (`rm_vision`)

视觉流水线基于 [FYT/rm_vision](https://github.com/FYT-PB2025-dong/rm_vision) 框架构建：

- **相机驱动** (`hik_camera`)：通过官方 MVS SDK 支持海康 MV-CS 系列相机，发布 `/camera/image_raw` 及标定信息。
- **装甲板检测** (`armor_detector`)：基于 Intel OpenVINO 异步推理的轻量级 CNN（YOLOX-tiny），检测延迟 < 5 ms。
- **装甲板跟踪** (`armor_tracker`)：自适应 EKF，在三维空间内估计目标装甲板位姿与速度，支持目标切换和陀螺装甲识别。
- **弹道解算** (`ballistic_solver`)：迭代重力补偿弹道解算器，输出修正后的俯仰/偏航云台指令。

### 2 · 导航与建图 (`pb2025_sentry_nav`)

- **Point-LIO**：固态/机械旋转激光雷达的直接积分三维里程计，无需扫描匹配，以 ≥ 100 Hz 提供鲁棒位姿估计。
- **Nav2 栈**：Costmap2D、AMCL 定位、Smac 规划器和 DWB 局部规划器，针对哨兵底盘动力学进行了调优。

### 3 · 自主巡逻 (`pb2025_nav2_patrol`)

使用 Nav2 的 `NavigateThroughPoses` 动作实现可配置的路点巡逻循环，支持随机、顺序、威胁响应等多种巡逻策略的运行时切换。

### 4 · 决策与行为 (`pb2025_sentry_behavior`)

基于 BehaviorTree.CPP v4 的决策引擎，包含自定义节点：

| 节点 | 类型 | 描述 |
|------|------|------|
| `CheckRobotAlive` | Condition | 从硬件桥接查询血量 |
| `CheckEnemyDetected` | Condition | 查询视觉流水线 |
| `AutoAimShoot` | Action | 启用云台自瞄与射击 |
| `PatrolNavigate` | Action | 向 Nav2 发送巡逻目标 |
| `EmergencyReturn` | Action | 导航至安全区域 |

### 5 · 硬件通信 (`standard_robot_pp_ros2`)

ROS 2 与基于 STM32 的底盘/云台 MCU 之间的串口/CAN 桥接，实现标准竞赛通信协议：
- 接收：机器人血量、比赛状态、敌方位置（来自裁判系统）。
- 发送：云台角度指令、射击使能、底盘速度覆写。

### 6 · 仿真环境 (`rmu_gazebo_simulator`)

2025 RoboMaster 比赛场地的完整物理仿真，包含：
- 具有差速/轮式底盘的哨兵机器人精确 URDF。
- 相机、IMU 和激光雷达传感器仿真的 Gazebo 插件。
- 裁判系统血量和比赛状态模拟节点。

---

## 🙏 致谢 / 基于以下开源项目

本项目建立在以下优秀开源贡献的基础之上，我们向每一个团队和作者表示深切的感谢：

| 项目 | 贡献者 | 在本仓库中的应用 |
|------|--------|----------------|
| [rmoss / RoboMaster OSS](https://github.com/robomaster-oss) | RoboMaster OSS 社区 | 核心 ROS 2 接口、消息定义与工具链规范 |
| [TUP-Infantry-Vision](https://github.com/tup-robomaster) | 清华大学 (TUP) 步兵视觉组 | YOLOX 装甲板检测模型权重与训练流水线 |
| [OpenVINO 异步推理](https://github.com/ev3rm0re/Vision) | 四川大学 OpenVINO 视觉团队 | 用于实时检测器的异步 OpenVINO 推理封装 |
| [rm.cv.fans / 自适应 EKF](https://github.com/julyfun/rm.cv.fans) | 上海交通大学 (julyfun) | 用于装甲板目标跟踪的自适应扩展卡尔曼滤波器设计 |
| [Point-LIO](https://github.com/hku-mars/Point-LIO) | 香港大学 MARS 实验室 | 点级激光雷达惯性里程计，用于三维建图 |
| [Nav2](https://github.com/ros-navigation/navigation2) | Open Robotics / Nav2 社区 | Navigation2 行为树、规划器与代价地图 |

> 如有遗漏的致谢，请提交 Issue，我们将立即更正。

---

## 📜 开源协议

本项目以 [MIT 协议](LICENSE) 开源。第三方组件保留其各自的许可证——请参阅每个子模块了解详情。

---

## 🤝 贡献指南

欢迎贡献代码、提交 Issue 和功能请求！请在提交 Pull Request 前阅读 [CONTRIBUTING.md](CONTRIBUTING.md)。

1. Fork 本仓库。
2. 创建特性分支：`git checkout -b feat/your-feature`。
3. 使用描述性的提交信息提交更改。
4. 向 `main` 分支发起 Pull Request。

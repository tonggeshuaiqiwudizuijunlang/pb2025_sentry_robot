# PB2025 RM Sentry Robot ROS2 Workspace 🤖🎯
[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## 🌟 AI & Robotics Synergy | 机器人与 AI 深度融合
本项目深度探索了 **移动机器人 (Motion & Mobile)** 与 AI 技术的结合，针对高性能竞技场景进行了专项优化：
- **全自主决策链 (Long-chain Reasoning)**：采用行为树架构，实现多传感器信息融合后的长链条逻辑推理，模拟人类战术层面的决策过程。
- **端侧实时 AI 推理**：针对边缘计算设备优化的视觉算法，实现在复杂动态背景下的高频目标检测与运动轨迹预测。
- **模块化可扩展架构**：全方位兼容主流 AI 工具链，为竞技机器人的智能化转型提供标准化、可落地的算法参考。

> 本项目为 **RoboMaster 2025 赛季** 全自动哨兵机器人（Sentry）的完整底层与算法架构。基于 ROS 2 Humble 开发，集成了高频视觉自瞄、3D 激光雷达建图与导航、行为树决策以及底层全向控制。

## ✨ 核心特性 / Features

*   🎯 **高性能自瞄系统 (Auto-Aim)**
    *   基于 YOLO 的装甲板检测与关键点提取。
    *   基于自适应扩展卡尔曼滤波 (Adaptive EKF) 的装甲板追踪预测。
    *   动态弹道补偿模型（支持无阻力理想模型与空气阻力模型切换）。
*   🗺️ **3D SLAM 与自主导航 (Nav & Patrol)**
    *   深度集成的 Point-LIO 建图与高频里程计 (`pb2025_sentry_nav`)。
    *   基于 Nav2 的自定义巡逻节点与全局路径规划 (`pb2025_nav2_patrol`)。
*   🧠 **行为树决策中枢 (Behavior Tree)**
    *   采用 `Nav2 Behavior Tree` 构建的顶级逻辑框架 (`pb2025_sentry_behavior`)。
    *   支持状态自检、自瞄与导航优先级动态切换、自动射击控制 (`pub_shoot`)。
*   🔌 **高实时性硬件接口 (Hardware Interface)**
    *   标准底层通讯协议解析 (`standard_robot_pp_ros2`)，实现与下位机的低延迟串口通讯。

## 📁 目录结构 / Directory Structure

```text
src/
├── FYT/                      # 视觉自瞄相关模块 (相机驱动, 检测, 预测)
├── pb2025_sentry_nav/        # 导航与建图模块 (含 Point-LIO)
├── pb2025_nav2_patrol/       # 哨兵自主巡逻节点
├── pb2025_sentry_behavior/   # 行为树决策逻辑 (行为插件, 条件插件)
├── pb2025_sentry_bringup/    # 机器人一键启动 Launch 脚本
├── pb2025_robot_description/ # 机器人的 URDF/Xacro 模型定义
├── standard_robot_pp_ros2/   # 上下位机底层通讯包
├── rmu_gazebo_simulator/     # RMU 场地及机器人 Gazebo 仿真环境
└── tools/                    # 调试工具与脚本 (TF树生成, 弹道可视化等)
```

## 🚀 快速启动 / Quick Start

### 1. 依赖安装
请确保您的环境为 **Ubuntu 22.04** 并已安装 **ROS 2 Humble**。
```bash
# 这里假设工作空间名为 pb2025_sentry_ws
cd ~/pb2025_sentry_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 2. 编译
建议分块编译以保证内存不溢出：
```bash
colcon build --packages-select auto_aim_interfaces interfaces
colcon build --packages-select armor_detector armor_tracker hik_camera rm_vision_bringup
colcon build --packages-select standard_robot_pp_ros2 pb2025_sentry_behavior
```

### 3. 一键运行
*   启动底层通讯与导航:
    ```bash
    ros2 launch pb2025_sentry_bringup rm_navigation_reality_launch.py slam:=False
    ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py
    ```
*   启动视觉自瞄系统:
    ```bash
    ros2 launch rm_vision_bringup vision_bringup.launch.py
    ```

## 🛠️ 调试指南 / Debugging
本项目提供了详尽的调试文档供二次开发参考：
*   [烧饼常用指令与自启动指南](./烧饼常用指令.md)
*   [自瞄系统调试指南](./自瞄系统调试指南.md)

## 🙏 鸣谢与开源声明 / Acknowledgements & Based On

本项目的开发站在了巨人的肩膀上，部分核心算法与框架深度参考并二次开发了以下优秀的 RoboMaster 社区开源成果，在此向各位开源先驱致以最诚挚的敬意：

*   **[rmoss_core](https://github.com/robomaster-oss/rmoss_core)**: RoboMaster 社区提供的通用基础模块。
*   **[TUP-InfantryVision-2022](https://github.com/tup-robomaster/TUP-InfantryVision-2022)** & **[TUP-NN-Train-2](https://github.com/tup-robomaster/TUP-NN-Train-2)**: 感谢清华大学（TUP）战队提供的视觉识别算法参考与 YOLOX 关键点模型训练框架。
*   **[rm_vision-OpenVINO](https://github.com/Ericsii/rm_vision-OpenVINO)**: 感谢四川大学提供的 OpenVINO 异步推理与部署方案。
*   **[rm.cv.fans](https://github.com/julyfun/rm.cv.fans/tree/main)**: 感谢上海交通大学交龙战队提供的自适应扩展卡尔曼滤波 (Adaptive EKF) 推导参考。
*   **[Point-LIO](https://github.com/hku-mars/Point-LIO)**: 感谢香港大学 MARS 实验室提供的高鲁棒性 LiDAR 里程计方案。
*   **[Nav2](https://navigation.ros.org/)**: 感谢 ROS 2 社区提供的 Navigation 2 及其 Behavior Tree 行为树架构支持。

> 饮水思源，欢迎各个战队交流学习！

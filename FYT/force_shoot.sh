#!/bin/bash

echo "======================================================"
echo "开始强制发送防抖/测试射击指令 (fire_advice = true)..."
echo "按 Ctrl+C 停止发送"
echo "======================================================"

# 以 10Hz 的频率循环发布强制射击指令。
# 当你需要在调试震荡时强行让程序认为“已瞄准好并开始射击”时运行它
ros2 topic pub --rate 20 /armor_solver/cmd_gimbal rm_interfaces/msg/GimbalCmd "{
  fire_advice: true,
  yaw_diff: 0.0,
  pitch_diff: 0.0,
  distance: 1.0,
  yaw: 0.0,
  pitch: 0.0
}"

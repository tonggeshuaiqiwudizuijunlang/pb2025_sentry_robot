#!/bin/bash
# watch_dog.sh

# 该脚本由 rm.service 启动，用于监控关键节点心跳并在崩溃时自动重启
# 脚本作者: Antigravity Support for NUC12WSKi7

TIMEOUT=10
NAMESPACE=""
# 监控具有 heartbeat 话题的核心节点
NODE_NAMES=("armor_detector" "armor_solver") 

USER="nuc"
HOME_DIR="/home/$USER"
# 工作空间根目录
WORKING_DIR="$HOME_DIR/pb2025_sentry_ws-main"
OUTPUT_FILE="$WORKING_DIR/screen.output"

function bringup() {
    source /opt/ros/humble/setup.bash
    source "$WORKING_DIR/install/setup.bash"
    
    echo "Starting Sentry 2025 system components at $(date)..."
    
    nohup ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py > "$WORKING_DIR/standard_robot.log" 2>&1 &
    # 1. FYT Vision (视觉识别)
    nohup ros2 launch pb2025_sentry_bringup fyt_vision.launch.py use_hik_camera:=True > "$WORKING_DIR/fyt_vision.log" 2>&1 &
    
    # 2. pb2025 Navigation (实车导航 - 关闭 rviz)
    nohup ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py slam:=True use_rviz:=False > "$WORKING_DIR/sentry_nav.log" 2>&1 &
    

    # 4. simple_sentry_control (两点往返控制)
    nohup ros2 launch simple_sentry_control simple_sentry_control.launch.py > "$WORKING_DIR/sentry_control.log" 2>&1 &
}

function check_alive() {
    for node_name in "${NODE_NAMES[@]}"; do
        topic="/$node_name/heartbeat"
        # 1. 检查话题是否存在
        topic_list=$(ros2 topic list)
        if [[ ! "$topic_list" =~ "$topic" ]]; then
            echo "Topic $topic not found!"
            return 1
        fi
        
        # 2. 检查心跳话题是否有值输出
        data_value=$(timeout 2 ros2 topic echo "$topic" --once | grep "data" | awk '{print $2}')
        if [ -z "$data_value" ]; then
            echo "Node $node_name heartbeat timeout!"
            return 1
        fi
    done
    return 0
}

# 确保启动环境干净
echo "Cleaning up before initial bringup..."
/usr/sbin/rm_clean_up.sh

while true; do
    # 检查核心节点心跳
    check_alive
    if [ $? -ne 0 ]; then
        echo "Nodes are down or heartbeat missing at $(date). Restarting system..."
        /usr/sbin/rm_clean_up.sh
        bringup
        sleep 20 # 预留更长的启动宽限期
    fi
    sleep $TIMEOUT
done

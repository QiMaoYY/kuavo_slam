#!/bin/bash
# Kuavo SLAM 建图启动脚本
# 功能：启动雷达驱动 -> 自动校准 -> 启动建图
# 用法：./start_mapping.sh [--no-calib]

set -e  # 遇到错误立即退出
# ============== 配置参数 ==============
SLAM_WS="/media/data/slam_ws"

# 默认需要校准
NEED_CALIBRATION=true

# 解析命令行参数
for arg in "$@"; do
    case $arg in
        --no-calib|--skip-calib)
            NEED_CALIBRATION=false
            shift
            ;;
    esac
done

# 激活conda环境
source ~/miniconda3/etc/profile.d/conda.sh
conda activate demo

source /opt/ros/noetic/setup.bash
source "${SLAM_WS}/devel/setup.bash"
source "${SLAM_WS}/livox_ws/devel/setup.bash"

# 自动校准（如果需要）
if [ "$NEED_CALIBRATION" = true ]; then
    python3 ${SLAM_WS}/livox_ws/src/livox_ros_driver2/scripts/mid360_autolevel_calib.py \
        --duration 3.0 \
        --timeout 10.0 \
        --imu-topic /livox/imu \
        --lidar-topic /livox/lidar \
        --fasterlio-yaml ${SLAM_WS}/src/faster-lio/config/
    if [ $? -ne 0 ]; then
        exit 1
    fi
fi

# 启动建图
roslaunch kuavo_slam mapping.launch rviz:=true visual_downsample_ratio:=100


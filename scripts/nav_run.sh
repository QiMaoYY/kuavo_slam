#!/bin/bash
# Kuavo SLAM 导航启动脚本
# 功能：启动环境 -> (可选)自动校准 -> 启动导航
# 用法：
#   ./nav_run.sh <MAP_NAME> <MAP_ROOT> <RVIZ:true|false> <CALIB:true|false>
# 例子：
#   ./nav_run.sh map_demo /media/data/slam_ws/src/kuavo_slam/maps true true

set -e  # 遇到错误立即退出
# ============== 配置参数 ==============
SLAM_WS="/media/data/slam_ws"

usage() {
  cat <<'EOF'
用法:
  ./nav_run.sh <MAP_NAME> <MAP_ROOT> <RVIZ:true|false> <CALIB:true|false>

参数:
  MAP_NAME   地图名 (默认: map_demo)
  MAP_ROOT   地图根目录 (默认: /media/data/slam_ws/src/kuavo_slam/maps)
  RVIZ       是否启动 RViz: true/false (默认: true)
  CALIB      是否执行自动校准: true/false (默认: true)

示例:
  ./nav_run.sh map_demo /media/data/slam_ws/src/kuavo_slam/maps true false
EOF
}

parse_bool() {
  # 输出: "true" 或 "false"
  # 接受: true/false/1/0/yes/no/on/off (不区分大小写)
  local v
  v="$(echo "${1:-}" | tr '[:upper:]' '[:lower:]')"
  case "$v" in
    true|1|yes|y|on)  echo "true" ;;
    false|0|no|n|off) echo "false" ;;
    *) return 1 ;;
  esac
}

# ============== 命令行参数（四个核心参数） ==============
MAP_NAME="${1:-map_demo}"
MAP_ROOT="${2:-/media/data/slam_ws/src/kuavo_slam/maps}"
RVIZ_RAW="${3:-true}"
CALIB_RAW="${4:-true}"

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
  usage
  exit 0
fi

if ! RVIZ="$(parse_bool "$RVIZ_RAW")"; then
  echo "错误: RVIZ 参数非法: '$RVIZ_RAW' (需要 true/false)" >&2
  usage
  exit 2
fi

if ! NEED_CALIBRATION="$(parse_bool "$CALIB_RAW")"; then
  echo "错误: CALIB 参数非法: '$CALIB_RAW' (需要 true/false)" >&2
  usage
  exit 2
fi

# 激活conda环境
source ~/miniconda3/etc/profile.d/conda.sh
conda activate demo

source /opt/ros/noetic/setup.bash
source ~/kuavo_data_pilot/devel/setup.bash
python3 /media/data/slam_ws/src/kuavo_slam/scripts/head.py

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
fi

# 启动导航
roslaunch kuavo_slam nav_run.launch rviz:=${RVIZ} map_path:=${MAP_ROOT} map_name:=${MAP_NAME}


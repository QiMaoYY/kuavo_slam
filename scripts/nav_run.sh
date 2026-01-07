#!/bin/bash
# Kuavo SLAM 导航启动脚本
# 功能：启动雷达驱动 -> 全局定位 -> 坐标系融合 -> 可视化
# 用法：./nav_run.sh

set -e  # 遇到错误立即退出

# ============== 配置参数 ==============
LIVOX_WS="/media/data/livox_ws"
SLAM_WS="/media/data/slam_ws"
MAP_SAVE_DIR="${SLAM_WS}/src/kuavo_slam/PCD"
FASTER_LIO_PCD_DIR="${SLAM_WS}/src/faster-lio/PCD"

# 默认需要校准
NEED_CALIBRATION=true

# 解析命令行参数
for arg in "$@"; do
    case $arg in
        --no-calib|--skip-calib)
            NEED_CALIBRATION=false
            shift
            ;;
        --help|-h)
            echo "用法: $0 [选项]"
            echo "选项:"
            echo "  --no-calib, --skip-calib    跳过雷达校准步骤"
            echo "  --help, -h                  显示此帮助信息"
            exit 0
            ;;
    esac
done

# ============== 颜色输出 ==============
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ============== 函数定义 ==============
info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 清理函数
cleanup() {
    info "正在清理并关闭所有节点..."
    
    # 温和地结束进程（给它们时间保存数据）
    info "  停止建图节点..."
    killall -INT run_mapping_online 2>/dev/null || true
    sleep 1
    
    info "  停止雷达驱动..."
    killall -INT roslaunch 2>/dev/null || true
    sleep 1
    
    info "  停止可视化..."
    killall -INT rviz 2>/dev/null || true
    sleep 1
    
    # 如果还有残留，强制杀死
    killall -9 roslaunch 2>/dev/null || true
    killall -9 rviz 2>/dev/null || true
    killall -9 run_mapping_online 2>/dev/null || true
    
    # 清理 ROS 节点（设置超时避免卡死）
    info "  清理 ROS 节点..."
    timeout 3 rosnode kill -a 2>/dev/null || true
    
    # 关闭 rosmaster（如果是脚本启动的）
    killall -9 rosmaster 2>/dev/null || true
    killall -9 roscore 2>/dev/null || true
    
    success "清理完成"
}

# 中断处理函数
handle_interrupt() {
    echo ""
    warn "检测到中断信号 (Ctrl+C)，正在安全关闭..."
    
    # 停止所有后台进程
    killall -INT roslaunch 2>/dev/null || true
    sleep 2
    
    
    # 清理
    cleanup
    
    echo ""
    success "========================================="
    success "  导航流程完成！"
    success "========================================="
    
    exit 0
}


# ============== 主流程 ==============
main() {
    echo ""
    info "========================================="
    info "   Kuavo SLAM 导航启动脚本"
    info "========================================="
    echo ""
    
    # 1. 自动加载 ROS 环境
    if [ -z "$ROS_ROOT" ]; then
        info "ROS 环境未初始化，正在自动加载..."
        if [ -f "/opt/ros/noetic/setup.bash" ]; then
            source /opt/ros/noetic/setup.bash
        elif [ -f "/opt/ros/melodic/setup.bash" ]; then
            source /opt/ros/melodic/setup.bash
        else
            error "未找到 ROS 安装，请手动 source ROS 环境"
            exit 1
        fi
    fi
    
    # 2. Source 工作空间
    info "加载 SLAM 工作空间..."
    if [ -f "${SLAM_WS}/devel/setup.bash" ]; then
        source "${SLAM_WS}/devel/setup.bash"
    else
        error "未找到 SLAM 工作空间: ${SLAM_WS}/devel/setup.bash"
        exit 1
    fi
    
    info "加载 Livox 工作空间..."
    if [ -f "${LIVOX_WS}/devel/setup.bash" ]; then
        source "${LIVOX_WS}/devel/setup.bash"
    else
        warn "未找到 Livox 工作空间: ${LIVOX_WS}/devel/setup.bash"
        info "将使用系统中已安装的 livox_ros_driver2"
    fi
    
    # 3. 启动 roscore（如果未运行）
    if ! rostopic list &>/dev/null; then
        info "启动 roscore..."
        roscore &
        sleep 3
    fi

    # 4. 自动校准（如果需要）
    if [ "$NEED_CALIBRATION" = true ]; then
        info "========================================="
        info "  开始自动校准 MID360 雷达"
        info "========================================="
        python3 ${LIVOX_WS}/src/livox_ros_driver2/scripts/mid360_autolevel_calib.py \
            --duration 3.0 \
            --timeout 10.0 \
            --imu-topic /livox/imu \
            --lidar-topic /livox/lidar \
            --fasterlio-yaml ${SLAM_WS}/src/faster-lio/config/mid360.yaml
        
        if [ $? -eq 0 ]; then
            success "自动校准完成！"
        else
            error "校准失败，请检查雷达连接"
            exit 1
        fi
        
        info "========================================="
        echo ""
    else
        warn "跳过校准步骤（使用已有外参）"
        echo ""
    fi

    # 5. 启动导航
    info "启动导航..."
    roslaunch kuavo_slam nav_run.launch \
        rviz:=true \
        &
    NAV_PID=$!
    
    echo ""
    success "导航已启动！"
    info "========================================="
    echo ""
    
    # 6. 等待用户中断（Ctrl+C）
    trap handle_interrupt SIGINT SIGTERM
    
    echo ""
    info "导航正在进行中..."
    info "按 Ctrl+C 停止导航"
    echo ""
    
    # 持续等待（直到用户按 Ctrl+C 或进程结束）
    while kill -0 $NAV_PID 2>/dev/null; do
        sleep 1
    done
    
    # 7. 如果进程自然结束（未被中断），执行保存
    echo ""
    info "导航进程已结束"
    sleep 3  # 等待文件写入完成
    
    # 8. 清理
    cleanup
    
    echo ""
    success "========================================="
    success "  导航流程完成！"
    success "========================================="
}

# ============== 脚本入口 ==============
main "$@"


#!/bin/bash
# 手动保存地图脚本
# 功能：将 faster-lio 建图生成的 PCD 文件移动到 kuavo_slam/PCD 目录

set -e

# ============== 配置 ==============
SLAM_WS="/media/data/slam_ws"
MAP_SAVE_DIR="${SLAM_WS}/src/kuavo_slam/PCD"
FASTER_LIO_PCD_DIR="${SLAM_WS}/src/faster-lio/PCD"

# 颜色
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

info() { echo -e "${BLUE}[INFO]${NC} $1"; }
success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }

# ============== 主流程 ==============
info "正在保存地图文件..."

# 检查源目录
if [ ! -d "${FASTER_LIO_PCD_DIR}" ]; then
    warn "源目录不存在: ${FASTER_LIO_PCD_DIR}"
    exit 1
fi

# 创建目标目录
mkdir -p "${MAP_SAVE_DIR}"

# 查找 PCD 文件
pcd_files=$(find "${FASTER_LIO_PCD_DIR}" -maxdepth 1 -name "*.pcd" 2>/dev/null)

if [ -z "$pcd_files" ]; then
    warn "未找到任何 PCD 地图文件"
    exit 1
fi

info "找到以下地图文件："
echo "$pcd_files"
echo ""

# 移动文件
for pcd_file in $pcd_files; do
    filename=$(basename "$pcd_file")
    target_path="${MAP_SAVE_DIR}/${filename}"
    
    # 如果目标存在，备份
    if [ -f "$target_path" ]; then
        timestamp=$(date +%Y%m%d_%H%M%S)
        backup_path="${MAP_SAVE_DIR}/${filename%.pcd}_${timestamp}.pcd"
        warn "目标文件已存在，备份为: $(basename $backup_path)"
        mv "$target_path" "$backup_path"
    fi
    
    cp "$pcd_file" "$target_path"
    success "已复制: $filename -> ${MAP_SAVE_DIR}/"
done

echo ""
success "地图文件已保存到: ${MAP_SAVE_DIR}/"
info "原始文件保留在: ${FASTER_LIO_PCD_DIR}/"





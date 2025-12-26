# Kuavo SLAM 入口管理包

## 概述
`kuavo_slam` 是 Kuavo 机器人的 SLAM 统一入口管理包，整合了：
- **建图**：基于 Faster-LIO 的 3D 激光 SLAM 建图
- **定位**：基于 ICP 的全局定位与坐标融合
- **导航**：基于 move_base 的路径规划与避障

---

## 目录结构
```
kuavo_slam/
├── launch/
│   ├── mapping.launch        # 建图入口（自动管理雷达+建图）
│   └── nav_run.launch         # 导航定位入口
├── scripts/
│   ├── start_mapping.sh       # 建图 Shell 脚本（推荐）
│   ├── save_map.sh            # 手动保存地图脚本
│   ├── global_localization.py # ICP 全局定位
│   ├── transform_fusion.py    # 坐标系融合
│   └── publish_initial_pose.py
├── config/
│   └── mid360_nav.yaml        # 导航定位参数
├── maps/                      # 2D 栅格地图
├── nav_lidar/                 # move_base 导航参数
├── PCD/                       # 3D 点云地图（建图后自动保存到此）
└── rviz/                      # 可视化配置

```

---

## 使用指南

### 1️⃣ 建图（Mapping）

#### 方法 A：使用 Shell 脚本（推荐）
这是最简单和完整的方式，脚本会自动：
1. 启动雷达驱动并等待校准（12秒）
2. 启动 Faster-LIO 建图
3. 建图完成后自动保存 PCD 文件到 `kuavo_slam/PCD/`

**使用步骤：**
```bash
# 1. Source 工作空间
cd /media/data/slam_ws
source devel/setup.bash

# 2. 启动建图脚本
rosrun kuavo_slam start_mapping.sh

# 3. 移动机器人采集地图，完成后按 Ctrl+C 结束
# 脚本会自动保存地图文件
```

**配置参数（可在脚本内修改）：**
- `CALIBRATION_WAIT_TIME`：雷达校准等待时间，默认 12 秒
- `MAP_SAVE_DIR`：地图保存路径

---

#### 方法 B：使用 Launch 文件
如果你更习惯使用 `roslaunch`：

```bash
# 启动建图（会自动延迟启动 Faster-LIO）
roslaunch kuavo_slam mapping.launch

# 建图完成后，手动保存地图
rosrun kuavo_slam save_map.sh
```

**Launch 参数：**
- `rviz:=true/false` - 是否启动 RViz 可视化
- `calibration_delay:=12` - 雷达校准等待时间（秒）

---

#### 方法 C：手动分步启动
适合调试或自定义流程：

```bash
# 终端 1: 启动雷达驱动
source /media/data/livox_ws/devel/setup.bash
roslaunch livox_ros_driver2 autolevel_MID360.launch

# 等待 10-12 秒，直到校准完成

# 终端 2: 启动建图
source /media/data/slam_ws/devel/setup.bash
roslaunch faster_lio mapping_mid360.launch

# 建图完成后，保存地图
rosrun kuavo_slam save_map.sh
```

---

### 2️⃣ 定位与导航（Localization & Navigation）

```bash
# 启动导航定位（使用已保存的地图）
roslaunch kuavo_slam nav_run.launch
```

**功能：**
- 加载全局 PCD 点云地图
- 启动 ICP 定位
- 启动 move_base 导航
- 发布 2D 栅格地图用于路径规划

---

## 建图说明

### 地图保存位置
- **默认保存**: `faster-lio/PCD/scans.pcd`
- **最终位置**: `kuavo_slam/PCD/scans.pcd` （脚本自动移动）

### 雷达驱动参数说明

建图脚本自动设置的 Livox 驱动参数：

| 参数 | 值 | 说明 |
|------|-----|------|
| `xfer_format` | **1** | 点云格式：0=PointCloud2，**1=CustomMsg**（Faster-LIO 必需） |
| `output_type` | **0** | 输出方式：**0=发布到ROS**，1=只写bag文件（不发布topic） |

⚠️ **重要**：Faster-LIO 需要 `livox_ros_driver2/CustomMsg` 格式，必须设置 `xfer_format:=1`

### 建图配置修改
如需修改建图参数（如分辨率、保存间隔等），请编辑：
```bash
vim /media/data/slam_ws/src/faster-lio/config/mid360.yaml
```

关键参数：
```yaml
pcd_save:
    pcd_save_en: true      # 是否保存地图
    interval: -1           # -1=建图结束时一次性保存
                           # >0=每隔 N 帧保存一次（生成多个文件）
```

---

## 常见问题

### Q1: 雷达校准失败？
**解决方案：**
- 确保机器人静止放置在平稳地面
- 增加 `calibration_delay` 参数到 15 秒
- 检查雷达供电和网络连接

### Q2: 未找到 PCD 文件？
**原因：**
- 建图时间过短，未生成有效地图
- `pcd_save_en` 参数未开启

**解决方案：**
```bash
# 检查配置
cat /media/data/slam_ws/src/faster-lio/config/mid360.yaml | grep pcd_save_en

# 应该输出: pcd_save_en: true
```

### Q3: 如何查看已保存的地图？
```bash
# 使用 pcl_viewer 查看点云
pcl_viewer /media/data/slam_ws/src/kuavo_slam/PCD/scans.pcd

# 或在 RViz 中加载
rosrun pcl_ros pcd_to_pointcloud /media/data/slam_ws/src/kuavo_slam/PCD/scans.pcd 0.1 _frame_id:=world
```

---

## 技术说明

### 雷达驱动
- **包名**: `livox_ros_driver2`
- **Launch**: `autolevel_MID360.launch`
- **功能**: 自动水平校准、IMU-LiDAR 外参标定
- **校准时间**: 约 8-12 秒

### 建图算法
- **算法**: Faster-LIO (基于 FAST-LIO 改进)
- **特点**: 实时性强、精度高、适用于 Livox 固态雷达
- **输出**: 3D 点云地图 (.pcd 格式)

### 定位算法
- **算法**: ICP (Iterative Closest Point)
- **输入**: 实时点云 + 全局 PCD 地图
- **输出**: 机器人在世界坐标系下的位姿

---

## 依赖包
- `faster_lio` - 建图核心
- `livox_ros_driver2` - 雷达驱动
- `move_base` - 导航规划
- `pcl_ros` - 点云处理
- `map_server` - 地图服务
- `pointcloud_to_laserscan` - 点云转激光

---

## 维护者
- 请根据实际情况填写

## 许可证
MIT License


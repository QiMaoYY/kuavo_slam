#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
可视化点云降采样节点
功能：对 /cloud_registered 话题进行降采样后发布到 /cloud_registered_visual
用途：减少 RViz 显示负载，避免建图时卡顿
"""

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from collections import deque


class VisualDownsampler:
    def __init__(self):
        rospy.init_node('visual_downsample', anonymous=False)
        
        # 参数
        self.downsample_ratio = rospy.get_param('~downsample_ratio', 50)  # 降采样比例 N:1
        self.queue_size = rospy.get_param('~queue_size', 10)  # 订阅队列大小
        
        rospy.loginfo(f"可视化降采样节点启动 - 降采样比例: {self.downsample_ratio}:1")
        
        # 发布器
        self.pub = rospy.Publisher('/cloud_registered_visual', PointCloud2, queue_size=10)
        
        # 订阅器
        self.sub = rospy.Subscriber('/cloud_registered', PointCloud2, self.callback, queue_size=self.queue_size)
        
        # 统计信息
        self.msg_count = 0
        self.last_log_time = rospy.Time.now()
        
    def callback(self, msg):
        """点云降采样回调函数"""
        try:
            # 读取点云数据
            points = []
            for point in pc2.read_points(msg, skip_nans=True):
                points.append(point)
            
            if len(points) == 0:
                return
            
            # 降采样：每 N 个点取一个
            downsampled_points = points[::self.downsample_ratio]
            
            # 创建降采样后的点云消息
            header = msg.header
            fields = msg.fields
            
            # 发布降采样后的点云
            cloud_msg = pc2.create_cloud(header, fields, downsampled_points)
            self.pub.publish(cloud_msg)
            
            # 统计信息（每5秒打印一次）
            self.msg_count += 1
            current_time = rospy.Time.now()
            if (current_time - self.last_log_time).to_sec() >= 5.0:
                rospy.loginfo(f"降采样统计 - 原始点数: {len(points)}, 降采样后: {len(downsampled_points)}, "
                             f"压缩率: {100 * (1 - len(downsampled_points)/len(points)):.1f}%")
                self.last_log_time = current_time
                
        except Exception as e:
            rospy.logerr(f"降采样处理失败: {e}")
    
    def run(self):
        """运行节点"""
        rospy.loginfo("可视化降采样节点正在运行...")
        rospy.spin()


if __name__ == '__main__':
    try:
        node = VisualDownsampler()
        node.run()
    except rospy.ROSInterruptException:
        pass


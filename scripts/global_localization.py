#!/usr/bin/python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import _thread
from re import T
import time

import open3d as o3d
import rospy
import ros_numpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
import tf
import tf.transformations

global_map = None
initialized = False
T_map_to_odom = np.eye(4)
cur_odom = None
cur_scan = None


def pose_to_mat(pose_msg: PoseWithCovarianceStamped):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )


def msg_to_array(pc_msg):
    pc_array = ros_numpy.numpify(pc_msg)
    pc = np.zeros([len(pc_array), 3])
    pc[:, 0] = pc_array['x']
    pc[:, 1] = pc_array['y']
    pc[:, 2] = pc_array['z']
    return pc


def registration_at_scale(pc_scan, pc_map, initial, scale):
    result_icp = o3d.pipelines.registration.registration_icp(
        voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale), voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale),
        1.0 * scale, initial,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=40)
    )

    return result_icp.transformation, result_icp.fitness


def inverse_se3(trans):
    trans_inverse = np.eye(4)
    # R
    trans_inverse[:3, :3] = trans[:3, :3].T
    # t
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse


def publish_point_cloud(publisher, header, pc):
    data = np.zeros(len(pc), dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32),
    ])
    data['x'] = pc[:, 0]
    data['y'] = pc[:, 1]
    data['z'] = pc[:, 2]
    if pc.shape[1] == 4:
        data['intensity'] = pc[:, 3]
    msg = ros_numpy.msgify(PointCloud2, data)
    msg.header = header
    publisher.publish(msg)


def crop_global_map_in_FOV(global_map, pose_estimation, cur_odom, cur_scan):

    # 当前scan原点的位姿
    T_odom_to_base_link = pose_to_mat(cur_odom)
    T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
    T_base_link_to_map = inverse_se3(T_map_to_base_link)
    # 把地图转换到lidar系下
    global_map_in_map = np.array(global_map.points)
    global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
    global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

    # 分别统计当前扫描点云在xyz三个轴方向的分布范围
    # 对每个轴计算百分位数范围，用于动态确定有效区域
    curscan_pc = o3d.geometry.PointCloud()
    curscan_pc.points = o3d.utility.Vector3dVector(cur_scan[:, :3])
    curscan_pc = voxel_down_sample(curscan_pc, SCAN_VOXEL_SIZE)
    cur_scan_down = np.array(curscan_pc.points)
    # 计算当前扫描点云的中心点
    scan_x = cur_scan_down[:, 0]
    scan_y = cur_scan_down[:, 1]
    
    # 计算每个轴的百分位数范围（例如5%到95%）
    x_min = np.percentile(scan_x, (100 - DISTANCE_PERCENTILE) / 2)
    x_max = np.percentile(scan_x, 100 - (100 - DISTANCE_PERCENTILE) / 2)
    y_min = np.percentile(scan_y, (100 - DISTANCE_PERCENTILE) / 2)
    y_max = np.percentile(scan_y, 100 - (100 - DISTANCE_PERCENTILE) / 2)
    
    # 添加安全边界
    x_range = x_max - x_min
    y_range = y_max - y_min
    
    x_min_margin = x_min - x_range * (DISTANCE_MARGIN - 1.0)
    x_max_margin = x_max + x_range * (DISTANCE_MARGIN - 1.0)
    y_min_margin = y_min - y_range * (DISTANCE_MARGIN - 1.0)
    y_max_margin = y_max + y_range * (DISTANCE_MARGIN - 1.0)
    
    
    rospy.loginfo_throttle(5.0, 
        'Scan range stats ({}%ile with margin {:.1f}): '
        'x=[{:.2f}, {:.2f}]m, y=[{:.2f}, {:.2f}]m'.format(
        DISTANCE_PERCENTILE, DISTANCE_MARGIN,
        x_min_margin, x_max_margin, y_min_margin, y_max_margin))

    # 根据xyz三个轴的范围过滤地图点
    if FOV > 3.14:
        # 环状lidar，同时考虑FOV角度限制和xyz范围
        indices = np.where(
            (global_map_in_base_link[:, 0] >= x_min_margin) &
            (global_map_in_base_link[:, 0] <= x_max_margin) &
            (global_map_in_base_link[:, 1] >= y_min_margin) &
            (global_map_in_base_link[:, 1] <= y_max_margin) )
    else:
        # 非环状lidar，同时考虑FOV角度限制和xyz范围
        indices = np.where(
            (global_map_in_base_link[:, 0] >= x_min_margin) &
            (global_map_in_base_link[:, 0] <= x_max_margin) &
            (global_map_in_base_link[:, 1] >= y_min_margin) &
            (global_map_in_base_link[:, 1] <= y_max_margin) )
    
    global_map_in_FOV = o3d.geometry.PointCloud()
    global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

    # 发布fov内点云
    header = cur_odom.header
    header.frame_id = 'world'
    publish_point_cloud(pub_submap, header, np.array(global_map_in_FOV.points)[::10])

    return global_map_in_FOV


def global_localization(map2odom):
    global global_map, cur_scan, cur_odom, T_map_to_odom
    # 用icp配准
    # print(global_map, cur_scan, T_map_to_odom)
    rospy.loginfo('Matching global map......')

    # TODO 这里注意线程安全
    scan_in_baselink = copy.copy(cur_scan)
    # 坐标变换，需要将base_link系下的scan坐标变换到map系下
    T_odom_to_base_link = pose_to_mat(cur_odom)
    T_map_to_base_link = np.matmul(map2odom, T_odom_to_base_link)
    scan_in_baselink = np.column_stack([scan_in_baselink, np.ones(len(scan_in_baselink))])
    scan_in_map = np.matmul(T_map_to_base_link, scan_in_baselink.T).T
    scan_tobe_mapped = o3d.geometry.PointCloud()
    scan_tobe_mapped.points = o3d.utility.Vector3dVector(scan_in_map[:, :3])

    tic = time.perf_counter()

    global_map_in_FOV = crop_global_map_in_FOV(global_map, map2odom, cur_odom, scan_in_baselink[:, :3])
    toc1 = time.perf_counter()
    # 粗配准
    transformation, _ = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=np.eye(4), scale=5)

    # 精配准
    transformation, fitness = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=transformation, scale=1)
    toc = time.perf_counter()
    rospy.loginfo('Use Time: crop: {}ms, icp: {}ms'.format((toc1 - tic)*1000, (toc - toc1)*1000))

    update_flag = True
    # 判断结果是否合法：旋转关系只可能出现绕着z轴的旋转，如果出现绕着x或y轴的旋转，则认为结果不合法
    # 检查旋转矩阵的第三行前两个元素和第三列前两个元素是否接近0
    if np.abs(transformation[2, 0]) > 0.1 or np.abs(transformation[2, 1]) > 0.1 or \
       np.abs(transformation[0, 2]) > 0.1 or np.abs(transformation[1, 2]) > 0.1:
        print('旋转关系不合法: 存在绕x或y轴的旋转')
        update_flag = False
    # 判断平移修正量距离或者旋转修正角度是否超过阈值，如果都小于阈值，则忽略本次修正结果
    # 从旋转矩阵中提取绕z轴的旋转角度
    roll, pitch, yaw = tf.transformations.euler_from_matrix(transformation)
    if np.linalg.norm(transformation[:3, 3]) < LOCALIZATION_DISTANCE_TH and np.abs(np.rad2deg(yaw)) < LOCALIZATION_ANGLE_TH:
        print('修正量过小: 平移={:.3f}m, 旋转={:.2f}度'.format(np.linalg.norm(transformation[:3, 3]), np.rad2deg(yaw)))
        update_flag = False

    # 当fitness大于LOCALIZATION_TH时且修正结果合法时才更新map2odom，或者初始化未完成时也更新map2odom
    if fitness > LOCALIZATION_TH and (update_flag == True or initialized == False):
        # T_map_to_odom = np.matmul(transformation, pose_estimation)
        T_map_to_odom = np.matmul(transformation, map2odom)

        # 发布map_to_odom
        map2odom = Odometry()
        xyz = tf.transformations.translation_from_matrix(T_map_to_odom)
        quat = tf.transformations.quaternion_from_matrix(T_map_to_odom)
        map2odom.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
        map2odom.header.stamp = cur_odom.header.stamp
        map2odom.header.frame_id = 'world'
        pub_map_to_odom.publish(map2odom)
        rospy.loginfo('fitness score:{}'.format(fitness))
        return True
    else:
        if fitness < LOCALIZATION_TH:
            rospy.logwarn('Not match!!!!')
        elif update_flag == False:
            rospy.logwarn('无需更新map2odom')
        # 发布map_to_odom
        map2odom = Odometry()
        xyz = tf.transformations.translation_from_matrix(T_map_to_odom)
        quat = tf.transformations.quaternion_from_matrix(T_map_to_odom)
        map2odom.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
        map2odom.header.stamp = cur_odom.header.stamp
        map2odom.header.frame_id = 'world'
        pub_map_to_odom.publish(map2odom)
        return False


def voxel_down_sample(pcd, voxel_size):
    try:
        pcd_down = pcd.voxel_down_sample(voxel_size)
    except:
        # for opend3d 0.7 or lower
        pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
    return pcd_down


def initialize_global_map(pc_msg):
    global global_map

    global_map = o3d.geometry.PointCloud()
    global_map.points = o3d.utility.Vector3dVector(msg_to_array(pc_msg)[:, :3])
    global_map = voxel_down_sample(global_map, MAP_VOXEL_SIZE)
    rospy.loginfo('Global map received.')


def cb_save_cur_odom(odom_msg):
    global cur_odom
    cur_odom = odom_msg


def cb_save_cur_scan(pc_msg):
    global cur_scan
    
    pc_msg.header.frame_id = 'base_link'
    pc_msg.header.stamp = rospy.Time().now()
    pub_pc_in_body.publish(pc_msg)
    # 转换为pcd
    # fastlio给的field有问题 处理一下
    pc_msg.fields = [pc_msg.fields[0], pc_msg.fields[1], pc_msg.fields[2],
                     pc_msg.fields[4], pc_msg.fields[5], pc_msg.fields[6],
                     pc_msg.fields[3], pc_msg.fields[7]]
    cur_scan = msg_to_array(pc_msg)


def thread_localization():
    global T_map_to_odom
    rate = rospy.Rate(FREQ_LOCALIZATION)
    while True:
        # 每隔一段时间进行全局定位

        # TODO
        global_localization(T_map_to_odom)
        rate.sleep()


if __name__ == '__main__':
    MAP_VOXEL_SIZE = 0.05
    SCAN_VOXEL_SIZE = 0.05

    # Global localization frequency (HZ)
    FREQ_LOCALIZATION = 0.5

    # The threshold of global localization,
    # only those scan2map-matching with higher fitness than LOCALIZATION_TH will be taken
    LOCALIZATION_TH = 0.95

    # FOV(rad), modify this according to your LiDAR type
    FOV = 3.14159

    # The farthest distance(meters) within FOV
    FOV_FAR = 15

    # 用于统计当前扫描点云距离的百分位数（0-100），用于动态确定有效距离范围
    # 例如：90表示取90%的点云所在的距离范围
    DISTANCE_PERCENTILE = 98

    # 距离安全边界系数，在百分位距离基础上乘以此系数，避免过滤掉有用的点
    # 例如：1.2表示在90%距离基础上增加20%的安全边界
    DISTANCE_MARGIN = 1.0

    # 定位修正量距离阈值(米)，用于过滤掉定位修正量距离过小的位姿
    LOCALIZATION_DISTANCE_TH = 0.15

    # 定位修正角度阈值(度)，用于过滤掉定位修正量角度过小的位姿
    LOCALIZATION_ANGLE_TH = 3.0

    rospy.init_node('fast_lio_localization')
    rospy.loginfo('Localization Node Inited...')

    # publisher
    pub_pc_in_body = rospy.Publisher('/cur_scan_in_map', PointCloud2, queue_size=1)
    pub_submap = rospy.Publisher('/submap', PointCloud2, queue_size=1)
    pub_map_to_odom = rospy.Publisher('/map_to_odom', Odometry, queue_size=1)

    rospy.Subscriber('/cloud_registered_body', PointCloud2, cb_save_cur_scan, queue_size=1)
    rospy.Subscriber('/odom', Odometry, cb_save_cur_odom, queue_size=1)

    # 初始化全局地图
    rospy.logwarn('Waiting for global map......')
    initialize_global_map(rospy.wait_for_message('/map', PointCloud2))

    # 初始化
    while not initialized:
        rospy.logwarn('Waiting for initial pose....')

        # 等待初始位姿
        pose_msg = rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
        if cur_odom is not None:
            initial_pose = pose_to_mat(pose_msg)
            initial_map_to_odom = np.matmul(initial_pose, inverse_se3(pose_to_mat(cur_odom)))
            if cur_scan is not None:
                initialized = global_localization(initial_map_to_odom)
            else:
                rospy.logwarn('First scan not received!!!!!')

    rospy.loginfo('')
    rospy.loginfo('Initialize successfully!!!!!!')
    rospy.loginfo('')
    # 开始定期全局定位
    _thread.start_new_thread(thread_localization, ())

    rospy.spin()

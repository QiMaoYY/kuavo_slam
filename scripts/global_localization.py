#!/usr/bin/python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import threading
import time
from collections import deque

import numpy as np
import open3d as o3d
import rospy
import ros_numpy
import tf
import tf.transformations
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

global_map = None
initialized = False
T_odom_to_map = np.eye(4)
cur_odom = None

scan_queue = None
last_scan_header = None

pending_initial_pose = None
force_relocalize = False
init_success_count = 0

submap_cache = None
submap_center = None

state_lock = threading.Lock()
kf_filters = {}

MAP_VOXEL_SIZE = 0.1
SCAN_VOXEL_SIZE = 0.2
FREQ_LOCALIZATION = 0.2
FITNESS_TH = 0.95
FITNESS_TH_INIT = 0.80
ROTATION_CHECK_TH = 0.1
LOCALIZATION_DISTANCE_TH = 0.05
LOCALIZATION_ANGLE_TH = 1.0
DISTANCE_PERCENTILE = 98
DISTANCE_MARGIN = 1.0
SUBMAP_MODE = 'fixed_box'
SUBMAP_EXTENT = [60.0, 60.0, 40.0]
SUBMAP_UPDATE_DISTANCE = 2.0
SUBMAP_PUB_STRIDE = 10
PUBLISH_SUBMAP = True
SCAN_QUEUE_SIZE = 5
SCAN_DOWNSAMPLE_IN_QUEUE = False
SCAN_MAX_POINTS = 0
MAX_POINTS_SOURCE = 80000
MAX_POINTS_TARGET = 400000
ICP_SCALES = [10, 6, 4, 1]
ICP_DISTANCE_FACTOR = 1.5
ICP_METHOD = 1
ICP_MAX_ITER = 40
INIT_SUCCESS_COUNT = 2
KF_ENABLE = True
KF_AXES = [False, False, True]
KF_PROCESS_VAR = [0.02, 0.02, 0.02]
KF_MEAS_VAR = [0.04, 0.04, 0.04]
MAP_FRAME = 'map'
ODOM_FRAME = 'odom'
BASE_LINK_FRAME = 'base_link'
SCAN_IN_ODOM = None
PUBLISH_DEBUG_SCAN = True
DEBUG_SCAN_STRIDE = 5
DEBUG_SCAN_TOPIC = '/cur_scan_in_map'

pub_pc_in_body = None
pub_submap = None
pub_odom_to_map = None


class KalmanFilter1D(object):
    def __init__(self, process_var, meas_var):
        self.process_var = process_var
        self.meas_var = meas_var
        self.posteri_est = None
        self.posteri_error = 1.0

    def reset(self, value=None):
        self.posteri_est = value
        self.posteri_error = 1.0

    def update(self, measurement):
        if self.posteri_est is None:
            self.posteri_est = measurement
            return measurement
        priori_est = self.posteri_est
        priori_error = self.posteri_error + self.process_var
        k_gain = priori_error / (priori_error + self.meas_var)
        self.posteri_est = priori_est + k_gain * (measurement - priori_est)
        self.posteri_error = (1.0 - k_gain) * priori_error
        return self.posteri_est


def to_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'y', 'on')
    return False


def normalize_3list(param, default_value, cast):
    if isinstance(param, (list, tuple)) and len(param) == 3:
        return [cast(x) for x in param]
    if isinstance(default_value, (list, tuple)) and len(default_value) == 3:
        return [cast(x) for x in default_value]
    return [cast(default_value) for _ in range(3)]


def reset_kalman_filters():
    kf_filters.clear()


def pose_to_mat(pose_msg):
    if hasattr(pose_msg, 'pose'):
        pose = pose_msg.pose
        if hasattr(pose, 'pose'):
            pose = pose.pose
    else:
        pose = pose_msg
    return np.matmul(
        tf.listener.xyz_to_mat44(pose.position),
        tf.listener.xyzw_to_mat44(pose.orientation),
    )


def msg_to_array(pc_msg):
    pc_array = ros_numpy.numpify(pc_msg)
    if len(pc_array) == 0:
        return np.empty((0, 3), dtype=np.float32)
    pc = np.zeros((len(pc_array), 3), dtype=np.float32)
    pc[:, 0] = pc_array['x']
    pc[:, 1] = pc_array['y']
    pc[:, 2] = pc_array['z']
    mask = np.isfinite(pc).all(axis=1)
    if not np.all(mask):
        pc = pc[mask]
    return pc


def transform_points(points, transform):
    if points is None or len(points) == 0:
        return points
    points_h = np.hstack([points[:, :3], np.ones((points.shape[0], 1))])
    transformed = np.matmul(transform, points_h.T).T
    return transformed[:, :3]


def inverse_se3(trans):
    trans_inverse = np.eye(4)
    trans_inverse[:3, :3] = trans[:3, :3].T
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse


def publish_point_cloud(publisher, header, pc):
    if publisher is None or pc is None or len(pc) == 0:
        return
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


def voxel_down_sample(pcd, voxel_size):
    if voxel_size <= 0:
        return pcd
    try:
        return pcd.voxel_down_sample(voxel_size)
    except AttributeError:
        return o3d.geometry.voxel_down_sample(pcd, voxel_size)


def limit_point_cloud(pcd, max_points):
    if max_points <= 0:
        return pcd
    num_points = np.asarray(pcd.points).shape[0]
    if num_points <= max_points:
        return pcd
    ratio = float(max_points) / float(num_points)
    try:
        return pcd.random_down_sample(ratio)
    except AttributeError:
        indices = np.random.choice(num_points, max_points, replace=False)
        return pcd.select_by_index(indices)


def estimate_normals(pcd, voxel_size):
    if len(pcd.points) == 0:
        return pcd
    radius = max(voxel_size * 2.0, 0.05)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
    return pcd


def get_estimation_method():
    if ICP_METHOD == 1:
        return o3d.pipelines.registration.TransformationEstimationPointToPlane()
    if ICP_METHOD == 2:
        try:
            return o3d.pipelines.registration.TransformationEstimationForGeneralizedICP()
        except AttributeError:
            rospy.logwarn_once('Generalized ICP not supported, fallback to Point-to-Plane.')
            return o3d.pipelines.registration.TransformationEstimationPointToPlane()
    return o3d.pipelines.registration.TransformationEstimationPointToPoint()


def registration_multi_scale(scan_pcd, map_pcd, initial):
    transformation = initial
    fitness = 0.0
    base_voxel = max(SCAN_VOXEL_SIZE, MAP_VOXEL_SIZE)
    for scale in ICP_SCALES:
        scan_ds = voxel_down_sample(scan_pcd, SCAN_VOXEL_SIZE * scale)
        map_ds = voxel_down_sample(map_pcd, MAP_VOXEL_SIZE * scale)
        scan_ds = limit_point_cloud(scan_ds, MAX_POINTS_SOURCE)
        map_ds = limit_point_cloud(map_ds, MAX_POINTS_TARGET)

        if ICP_METHOD in (1, 2):
            estimate_normals(scan_ds, SCAN_VOXEL_SIZE * scale)
            estimate_normals(map_ds, MAP_VOXEL_SIZE * scale)

        dist_thresh = ICP_DISTANCE_FACTOR * base_voxel * scale
        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=ICP_MAX_ITER)
        result_icp = o3d.pipelines.registration.registration_icp(
            scan_ds, map_ds, dist_thresh, transformation, get_estimation_method(), criteria
        )
        transformation = result_icp.transformation
        fitness = result_icp.fitness
    return transformation, fitness


def get_base_link_in_map(odom2map, odom_msg):
    T_base_to_odom = pose_to_mat(odom_msg)
    T_base_to_map = np.matmul(odom2map, T_base_to_odom)
    return T_base_to_map


def crop_submap_fixed(global_map_pcd, odom2map, odom_msg):
    global submap_cache, submap_center
    T_base_to_map = get_base_link_in_map(odom2map, odom_msg)
    center = T_base_to_map[:3, 3]
    if submap_cache is not None and submap_center is not None:
        if np.linalg.norm(center - submap_center) < SUBMAP_UPDATE_DISTANCE:
            return submap_cache
    extent = np.array(SUBMAP_EXTENT, dtype=np.float64)
    min_bound = center - extent / 2.0
    max_bound = center + extent / 2.0
    aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    submap = global_map_pcd.crop(aabb)
    submap_cache = submap
    submap_center = center
    if PUBLISH_SUBMAP and len(submap.points) > 0:
        header = odom_msg.header
        header.frame_id = MAP_FRAME
        publish_point_cloud(pub_submap, header, np.asarray(submap.points)[::SUBMAP_PUB_STRIDE])
    return submap


def crop_submap_dynamic(global_map_pcd, odom2map, odom_msg, scan_in_odom):
    if scan_in_odom is None or len(scan_in_odom) == 0:
        return None
    T_base_to_odom = pose_to_mat(odom_msg)
    T_odom_to_base = inverse_se3(T_base_to_odom)
    scan_in_base = transform_points(scan_in_odom, T_odom_to_base)
    scan_pcd = o3d.geometry.PointCloud()
    scan_pcd.points = o3d.utility.Vector3dVector(scan_in_base[:, :3])
    scan_pcd = voxel_down_sample(scan_pcd, SCAN_VOXEL_SIZE)
    scan_down = np.asarray(scan_pcd.points)
    if scan_down.size == 0:
        return None

    scan_x = scan_down[:, 0]
    scan_y = scan_down[:, 1]
    x_min = np.percentile(scan_x, (100 - DISTANCE_PERCENTILE) / 2)
    x_max = np.percentile(scan_x, 100 - (100 - DISTANCE_PERCENTILE) / 2)
    y_min = np.percentile(scan_y, (100 - DISTANCE_PERCENTILE) / 2)
    y_max = np.percentile(scan_y, 100 - (100 - DISTANCE_PERCENTILE) / 2)

    x_range = x_max - x_min
    y_range = y_max - y_min
    x_min_margin = x_min - x_range * (DISTANCE_MARGIN - 1.0)
    x_max_margin = x_max + x_range * (DISTANCE_MARGIN - 1.0)
    y_min_margin = y_min - y_range * (DISTANCE_MARGIN - 1.0)
    y_max_margin = y_max + y_range * (DISTANCE_MARGIN - 1.0)

    rospy.loginfo_throttle(
        5.0,
        'Scan range stats ({}%ile with margin {:.1f}): x=[{:.2f}, {:.2f}]m, y=[{:.2f}, {:.2f}]m'.format(
            DISTANCE_PERCENTILE, DISTANCE_MARGIN,
            x_min_margin, x_max_margin, y_min_margin, y_max_margin)
    )

    T_base_to_map = get_base_link_in_map(odom2map, odom_msg)
    T_map_to_base = inverse_se3(T_base_to_map)
    global_map_np = np.asarray(global_map_pcd.points)
    global_map_in_base = transform_points(global_map_np, T_map_to_base)
    indices = np.where(
        (global_map_in_base[:, 0] >= x_min_margin) &
        (global_map_in_base[:, 0] <= x_max_margin) &
        (global_map_in_base[:, 1] >= y_min_margin) &
        (global_map_in_base[:, 1] <= y_max_margin)
    )
    submap = o3d.geometry.PointCloud()
    submap.points = o3d.utility.Vector3dVector(global_map_np[indices[0], :3])
    if PUBLISH_SUBMAP and len(submap.points) > 0:
        header = odom_msg.header
        header.frame_id = MAP_FRAME
        publish_point_cloud(pub_submap, header, np.asarray(submap.points)[::SUBMAP_PUB_STRIDE])
    return submap


def get_submap(odom2map, odom_msg, scan_in_odom):
    if SUBMAP_MODE == 'dynamic_scan':
        return crop_submap_dynamic(global_map, odom2map, odom_msg, scan_in_odom)
    return crop_submap_fixed(global_map, odom2map, odom_msg)


def build_odom_to_map_msg(odom2map, stamp):
    msg = Odometry()
    xyz = tf.transformations.translation_from_matrix(odom2map)
    quat = tf.transformations.quaternion_from_matrix(odom2map)
    msg.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
    msg.header.stamp = stamp
    msg.header.frame_id = MAP_FRAME
    msg.child_frame_id = ODOM_FRAME
    return msg


def publish_odom_to_map(odom2map, stamp):
    if pub_odom_to_map is None:
        return
    pub_odom_to_map.publish(build_odom_to_map_msg(odom2map, stamp))


def apply_kalman_filter(odom2map):
    if not KF_ENABLE:
        return odom2map
    filtered = odom2map.copy()
    translation = filtered[:3, 3].copy()
    axes = ['x', 'y', 'z']
    for idx, axis in enumerate(axes):
        if not KF_AXES[idx]:
            continue
        if axis not in kf_filters:
            kf_filters[axis] = KalmanFilter1D(KF_PROCESS_VAR[idx], KF_MEAS_VAR[idx])
        translation[idx] = kf_filters[axis].update(translation[idx])
    filtered[:3, 3] = translation
    return filtered


def is_rotation_valid(transform):
    return (
        np.abs(transform[2, 0]) <= ROTATION_CHECK_TH and
        np.abs(transform[2, 1]) <= ROTATION_CHECK_TH and
        np.abs(transform[0, 2]) <= ROTATION_CHECK_TH and
        np.abs(transform[1, 2]) <= ROTATION_CHECK_TH
    )


def is_update_significant(transform):
    _, _, yaw = tf.transformations.euler_from_matrix(transform)
    dist = np.linalg.norm(transform[:3, 3])
    angle = np.abs(np.rad2deg(yaw))
    return not (dist < LOCALIZATION_DISTANCE_TH and angle < LOCALIZATION_ANGLE_TH)


def get_accumulated_scan():
    with state_lock:
        if scan_queue is None or len(scan_queue) == 0:
            return None
        scans = list(scan_queue)
    return np.concatenate(scans, axis=0)


def global_localization(odom2map, scan_in_odom, odom_msg, is_initial=False):
    global T_odom_to_map
    if global_map is None or len(global_map.points) == 0:
        rospy.logwarn_throttle(5.0, 'Global map not ready.')
        return False
    if scan_in_odom is None or len(scan_in_odom) == 0:
        rospy.logwarn_throttle(5.0, 'Scan not ready.')
        return False
    if odom_msg is None:
        rospy.logwarn_throttle(5.0, 'Odometry not ready.')
        return False

    rospy.loginfo_throttle(2.0, 'Matching global map...')
    scan_in_map = transform_points(scan_in_odom, odom2map)
    scan_pcd = o3d.geometry.PointCloud()
    scan_pcd.points = o3d.utility.Vector3dVector(scan_in_map[:, :3])

    tic = time.perf_counter()
    submap = get_submap(odom2map, odom_msg, scan_in_odom)
    toc1 = time.perf_counter()
    if submap is None or len(submap.points) == 0:
        rospy.logwarn_throttle(5.0, 'Submap empty, skip.')
        return False

    transformation, fitness = registration_multi_scale(scan_pcd, submap, initial=np.eye(4))
    toc = time.perf_counter()
    rospy.loginfo('Use Time: crop: %.1fms, icp: %.1fms', (toc1 - tic) * 1000.0, (toc - toc1) * 1000.0)

    rotation_ok = is_rotation_valid(transformation)
    update_significant = is_update_significant(transformation)
    update_allowed = rotation_ok and (is_initial or update_significant)
    fitness_th = FITNESS_TH_INIT if is_initial else FITNESS_TH

    if fitness >= fitness_th and update_allowed:
        new_odom2map = np.matmul(transformation, odom2map)
        new_odom2map = apply_kalman_filter(new_odom2map)
        with state_lock:
            T_odom_to_map = new_odom2map
        publish_odom_to_map(new_odom2map, odom_msg.header.stamp)
        rospy.loginfo('fitness score: %.4f', fitness)
        return True

    if not rotation_ok:
        rospy.logwarn('Rotation invalid, skip update.')
    elif not update_significant and not is_initial:
        rospy.logwarn('Update too small, skip update.')
    else:
        rospy.logwarn('fitness score: %.4f, not match.', fitness)

    with state_lock:
        current_odom2map = T_odom_to_map.copy()
    #publish_odom_to_map(current_odom2map, odom_msg.header.stamp)
    return False


def initialize_global_map(pc_msg):
    global global_map, submap_cache, submap_center
    global_map = o3d.geometry.PointCloud()
    global_map.points = o3d.utility.Vector3dVector(msg_to_array(pc_msg)[:, :3])
    global_map = voxel_down_sample(global_map, MAP_VOXEL_SIZE)
    submap_cache = None
    submap_center = None
    rospy.loginfo('Global map received, points: %d', len(global_map.points))


def cb_save_cur_odom(odom_msg):
    global cur_odom
    with state_lock:
        cur_odom = odom_msg


def resolve_scan_in_odom(frame_id):
    frame_id = (frame_id or '').lstrip('/')
    odom_frame = ODOM_FRAME.lstrip('/')
    base_frame = BASE_LINK_FRAME.lstrip('/')
    if SCAN_IN_ODOM is None:
        if frame_id == odom_frame:
            return True
        if frame_id == base_frame:
            return False
        return False
    return bool(SCAN_IN_ODOM)


def cb_save_cur_scan(pc_msg):
    global last_scan_header
    # 转换为pcd
    # fastlio给的field有问题 处理一下
    pc_msg.fields = [pc_msg.fields[0], pc_msg.fields[1], pc_msg.fields[2],
                     pc_msg.fields[4], pc_msg.fields[5], pc_msg.fields[6],
                     pc_msg.fields[3], pc_msg.fields[7]]
    scan_points = msg_to_array(pc_msg)
    if scan_points is None or len(scan_points) == 0:
        return

    with state_lock:
        odom_msg = cur_odom
    if odom_msg is None:
        rospy.logwarn_throttle(5.0, 'Odometry not ready, skip scan.')
        return

    frame_id = (pc_msg.header.frame_id or BASE_LINK_FRAME).lstrip('/')
    scan_in_odom = resolve_scan_in_odom(frame_id)
    points_odom = scan_points

    if not scan_in_odom:
        if frame_id != BASE_LINK_FRAME:
            rospy.logwarn_throttle(5.0, 'Scan frame %s not in odom/base_link, treat as base_link.', frame_id)
        T_base_to_odom = pose_to_mat(odom_msg)
        points_odom = transform_points(scan_points, T_base_to_odom)

    if SCAN_DOWNSAMPLE_IN_QUEUE:
        scan_pcd = o3d.geometry.PointCloud()
        scan_pcd.points = o3d.utility.Vector3dVector(points_odom[:, :3])
        scan_pcd = voxel_down_sample(scan_pcd, SCAN_VOXEL_SIZE)
        points_odom = np.asarray(scan_pcd.points)

    if SCAN_MAX_POINTS > 0 and points_odom.shape[0] > SCAN_MAX_POINTS:
        indices = np.random.choice(points_odom.shape[0], SCAN_MAX_POINTS, replace=False)
        points_odom = points_odom[indices]

    with state_lock:
        if scan_queue is None:
            return
        scan_queue.append(points_odom)
        last_scan_header = pc_msg.header

    if PUBLISH_DEBUG_SCAN and pub_pc_in_body is not None:
        header = pc_msg.header
        header.frame_id = ODOM_FRAME
        publish_point_cloud(pub_pc_in_body, header, points_odom[::DEBUG_SCAN_STRIDE])


def cb_initialpose(pose_msg):
    global pending_initial_pose, force_relocalize, initialized, init_success_count, submap_cache, submap_center
    with state_lock:
        pending_initial_pose = pose_msg
        force_relocalize = True
        initialized = False
        init_success_count = 0
    submap_cache = None
    submap_center = None
    reset_kalman_filters()
    rospy.logwarn('Receive /initialpose, start relocalization.')
    seed_odom2map = initial_pose_to_odom2map(pending_initial_pose, cur_odom)
    publish_odom_to_map(seed_odom2map, cur_odom.header.stamp)


def initial_pose_to_odom2map(pose_msg, odom_msg):
    initial_pose = pose_to_mat(pose_msg)
    T_base_to_odom = pose_to_mat(odom_msg)
    return np.matmul(initial_pose, inverse_se3(T_base_to_odom))


def thread_localization():
    global init_success_count, initialized, force_relocalize, pending_initial_pose
    rate = rospy.Rate(FREQ_LOCALIZATION)
    while not rospy.is_shutdown():
        if global_map is None:
            rospy.logwarn_throttle(5.0, 'Waiting for global map...')
            rate.sleep()
            continue

        with state_lock:
            odom_msg = cur_odom
            odom2map = T_odom_to_map.copy()
            pending_pose = pending_initial_pose
            relocalize = force_relocalize
            current_init_success = init_success_count

        if odom_msg is None:
            rospy.logwarn_throttle(5.0, 'Waiting for odometry...')
            rate.sleep()
            continue

        scan_in_odom = get_accumulated_scan()
        if scan_in_odom is None or len(scan_in_odom) == 0:
            rospy.logwarn_throttle(5.0, 'Waiting for scan...')
            rate.sleep()
            continue

        if relocalize and pending_pose is not None:
            if current_init_success > 0:
                seed_odom2map = odom2map
            else:
                seed_odom2map = initial_pose_to_odom2map(pending_pose, odom_msg)
            success = global_localization(seed_odom2map, scan_in_odom, odom_msg, is_initial=True)
            if not success:
                success = global_localization(odom2map, scan_in_odom, odom_msg, is_initial=True)
            with state_lock:
                if success:
                    init_success_count += 1
                    if init_success_count >= INIT_SUCCESS_COUNT:
                        initialized = True
                        force_relocalize = False
                        pending_initial_pose = None
                        init_success_count = 0
                        rospy.loginfo('Initialize successfully.')
                else:
                    init_success_count = 0
        elif initialized:
            global_localization(odom2map, scan_in_odom, odom_msg, is_initial=False)
        else:
            rospy.logwarn_throttle(5.0, 'Waiting for initial pose...')
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('fast_lio_localization')
    rospy.loginfo('Localization Node Inited...')

    MAP_VOXEL_SIZE = rospy.get_param('~map_voxel', MAP_VOXEL_SIZE)
    SCAN_VOXEL_SIZE = rospy.get_param('~scan_voxel', SCAN_VOXEL_SIZE)
    FREQ_LOCALIZATION = rospy.get_param('~freq_localization', FREQ_LOCALIZATION)
    FITNESS_TH = rospy.get_param('~fitness_th', FITNESS_TH)
    FITNESS_TH_INIT = rospy.get_param('~fitness_th_init', FITNESS_TH_INIT)
    ROTATION_CHECK_TH = rospy.get_param('~rotation_check_th', ROTATION_CHECK_TH)
    LOCALIZATION_DISTANCE_TH = rospy.get_param('~min_update_dist', LOCALIZATION_DISTANCE_TH)
    LOCALIZATION_ANGLE_TH = rospy.get_param('~min_update_angle', LOCALIZATION_ANGLE_TH)
    DISTANCE_PERCENTILE = rospy.get_param('~distance_percentile', DISTANCE_PERCENTILE)
    DISTANCE_MARGIN = rospy.get_param('~distance_margin', DISTANCE_MARGIN)
    SUBMAP_MODE = rospy.get_param('~submap_mode', SUBMAP_MODE)
    SUBMAP_EXTENT = normalize_3list(rospy.get_param('~submap_extent', SUBMAP_EXTENT), SUBMAP_EXTENT, float)
    SUBMAP_UPDATE_DISTANCE = rospy.get_param('~submap_update_dist', SUBMAP_UPDATE_DISTANCE)
    SUBMAP_PUB_STRIDE = rospy.get_param('~submap_pub_stride', SUBMAP_PUB_STRIDE)
    PUBLISH_SUBMAP = to_bool(rospy.get_param('~publish_submap', PUBLISH_SUBMAP))
    SCAN_QUEUE_SIZE = int(rospy.get_param('~scan_queue_size', SCAN_QUEUE_SIZE))
    SCAN_DOWNSAMPLE_IN_QUEUE = to_bool(rospy.get_param('~scan_downsample_in_queue', SCAN_DOWNSAMPLE_IN_QUEUE))
    SCAN_MAX_POINTS = int(rospy.get_param('~scan_max_points', SCAN_MAX_POINTS))
    MAX_POINTS_SOURCE = int(rospy.get_param('~max_points_source', MAX_POINTS_SOURCE))
    MAX_POINTS_TARGET = int(rospy.get_param('~max_points_target', MAX_POINTS_TARGET))
    ICP_SCALES = rospy.get_param('~icp_scales', ICP_SCALES)
    ICP_DISTANCE_FACTOR = rospy.get_param('~icp_distance_factor', ICP_DISTANCE_FACTOR)
    ICP_METHOD = int(rospy.get_param('~icp_method', ICP_METHOD))
    ICP_MAX_ITER = int(rospy.get_param('~icp_max_iter', ICP_MAX_ITER))
    INIT_SUCCESS_COUNT = int(rospy.get_param('~init_success_count', INIT_SUCCESS_COUNT))
    KF_ENABLE = to_bool(rospy.get_param('~kf_enable', KF_ENABLE))
    KF_AXES = normalize_3list(rospy.get_param('~kf_axes', KF_AXES), KF_AXES, to_bool)
    KF_PROCESS_VAR = normalize_3list(rospy.get_param('~kf_process_var', KF_PROCESS_VAR), KF_PROCESS_VAR, float)
    KF_MEAS_VAR = normalize_3list(rospy.get_param('~kf_meas_var', KF_MEAS_VAR), KF_MEAS_VAR, float)
    MAP_FRAME = rospy.get_param('~map_frame', MAP_FRAME)
    ODOM_FRAME = rospy.get_param('~odom_frame', ODOM_FRAME)
    BASE_LINK_FRAME = rospy.get_param('~base_link_frame', BASE_LINK_FRAME)
    SCAN_IN_ODOM = rospy.get_param('~scan_in_odom', SCAN_IN_ODOM)
    if isinstance(SCAN_IN_ODOM, str) and SCAN_IN_ODOM.lower() == 'auto':
        SCAN_IN_ODOM = None
    PUBLISH_DEBUG_SCAN = to_bool(rospy.get_param('~publish_debug_scan', PUBLISH_DEBUG_SCAN))
    DEBUG_SCAN_STRIDE = int(rospy.get_param('~debug_scan_stride', DEBUG_SCAN_STRIDE))
    DEBUG_SCAN_TOPIC = rospy.get_param('~debug_scan_topic', DEBUG_SCAN_TOPIC)

    MAP_TOPIC = rospy.get_param('~map_topic', '/map')
    CLOUD_TOPIC = rospy.get_param('~cloud_topic', '/cloud_registered')
    ODOM_TOPIC = rospy.get_param('~odom_topic', '/Odometry')
    INITIALPOSE_TOPIC = rospy.get_param('~initialpose_topic', '/initialpose')

    if not isinstance(ICP_SCALES, (list, tuple)) or len(ICP_SCALES) == 0:
        ICP_SCALES = [6, 4, 1]
    ICP_SCALES = [int(scale) for scale in ICP_SCALES]
    if SCAN_QUEUE_SIZE < 1:
        SCAN_QUEUE_SIZE = 1

    scan_queue = deque(maxlen=SCAN_QUEUE_SIZE)

    pub_pc_in_body = rospy.Publisher(DEBUG_SCAN_TOPIC, PointCloud2, queue_size=1)
    pub_submap = rospy.Publisher('/submap', PointCloud2, queue_size=1)
    pub_odom_to_map = rospy.Publisher('/odom_to_map', Odometry, queue_size=1)

    rospy.Subscriber(CLOUD_TOPIC, PointCloud2, cb_save_cur_scan, queue_size=1)
    rospy.Subscriber(ODOM_TOPIC, Odometry, cb_save_cur_odom, queue_size=1)
    rospy.Subscriber(INITIALPOSE_TOPIC, PoseWithCovarianceStamped, cb_initialpose, queue_size=1)

    rospy.logwarn('Waiting for global map...')
    initialize_global_map(rospy.wait_for_message(MAP_TOPIC, PointCloud2))

    worker = threading.Thread(target=thread_localization)
    worker.daemon = True
    worker.start()

    rospy.spin()

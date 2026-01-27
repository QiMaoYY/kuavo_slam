#! /usr/bin/python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import _thread
import time

import numpy as np
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry

cur_odom_to_baselink = None
cur_odom_to_map = None


def pose_to_mat(pose_msg):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )


def transform_fusion():
    global cur_odom_to_baselink, cur_odom_to_map

    br = tf.TransformBroadcaster()

    # TODO 这里注意线程安全
    cur_odom = copy.copy(cur_odom_to_baselink)
    if cur_odom_to_map is not None:
        T_odom_to_map = pose_to_mat(cur_odom_to_map)
    else:
        return
    T_odom_to_base_link = pose_to_mat(cur_odom)
    br.sendTransform(tf.transformations.translation_from_matrix(T_odom_to_map),
                        tf.transformations.quaternion_from_matrix(T_odom_to_map),
                        rospy.Time.now(),
                        'odom', 'map')
    br.sendTransform(tf.transformations.translation_from_matrix(T_odom_to_base_link),
                        tf.transformations.quaternion_from_matrix(T_odom_to_base_link),
                        rospy.Time.now(),
                        'base_link', 'odom')
    if cur_odom is not None:
        # 发布全局定位的odometry
        localization = Odometry()
        T_odom_to_base_link = pose_to_mat(cur_odom)
        # 这里T_odom_to_map短时间内变化缓慢 暂时不考虑与T_odom_to_base_link时间同步
        T_map_to_base_link = np.matmul(T_odom_to_map, T_odom_to_base_link)
        xyz = tf.transformations.translation_from_matrix(T_map_to_base_link)
        quat = tf.transformations.quaternion_from_matrix(T_map_to_base_link)
        localization.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
        localization.twist = cur_odom.twist

        localization.header.stamp = cur_odom.header.stamp
        localization.header.frame_id = 'map'
        localization.child_frame_id = 'base_link'
        # rospy.loginfo_throttle(1, '{}'.format(np.matmul(T_odom_to_map, T_odom_to_base_link)))
        pub_localization.publish(localization)
    else:
        rospy.logwarn('cur_odom is None')
        return


def cb_save_cur_odom(odom_msg):
    global cur_odom_to_baselink
    cur_odom_to_baselink = odom_msg
    transform_fusion()


def cb_save_odom_to_map(odom_msg):
    global cur_odom_to_map
    cur_odom_to_map = odom_msg


if __name__ == '__main__':

    rospy.init_node('transform_fusion')
    rospy.loginfo('Transform Fusion Node Inited...')

    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)
    rospy.Subscriber('/odom_to_map', Odometry, cb_save_odom_to_map, queue_size=1)

    pub_localization = rospy.Publisher('/localization', Odometry, queue_size=1)


    rospy.spin()

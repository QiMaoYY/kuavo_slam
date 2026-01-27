#!/usr/bin/env python3
#coding: utf-8

import rospy
from geometry_msgs.msg import Twist
import sys
class CmdVelForwarder:
    def __init__(self, mode="pos"):
        # 初始化ROS节点
        rospy.init_node('cmd_vel_forwarder', anonymous=True)
        
        # 创建发布者，发布到/cmd_vel话题
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 创建订阅者，监听/cmd_vel_nav话题
        self.sub = rospy.Subscriber('/cmd_vel_'+mode, Twist, self.callback)
        
        rospy.loginfo(f"CMD_VEL转发节点已启动: /cmd_vel_{mode} -> /cmd_vel")
    
    def callback(self, msg):
        """接收到消息后转发到/cmd_vel话题"""
        self.pub.publish(msg)
        rospy.logdebug(f"转发消息: linear.x={msg.linear.x}, angular.z={msg.angular.z}")

def main():
    arg = sys.argv[1]
    CmdVelForwarder(arg)
    rospy.spin()

if __name__ == '__main__':
    main()
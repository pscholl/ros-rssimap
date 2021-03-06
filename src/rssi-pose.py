#!/usr/bin/env python

import roslib; roslib.load_manifest('rssimap')
import rospy, tf
from geometry_msgs.msg import PoseStamped, Pose
from rssimap.msg import RssiStamped, RssiPoseStamped
from numpy import arange

pub,lis = None,None

def rssi(data):
    try:
        t = lis.getLatestCommonTime("/world", "/base_link")
        (trans,rot) = lis.lookupTransform('/world', '/base_link', t)
        msg = RssiPoseStamped()
        msg.rssi = data
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/world"
        msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=trans
        msg.pose.orientation.x,msg.pose.orientation.y,\
                msg.pose.orientation.z,msg.pose.orientation.w=rot
        pub.publish(msg)
    except (Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

def rssi_pose_merge():
    global pub,lis
    rospy.init_node('rssi_pose')
    pub=rospy.Publisher('rssi_pose', RssiPoseStamped)
    lis=tf.TransformListener()
    rospy.Subscriber('rssi', RssiStamped, rssi)
    rospy.spin()

if __name__ == '__main__':
    try: rssi_pose_merge()
    except rospy.ROSInterruptException:
        pass

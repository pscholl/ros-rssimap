#!/usr/bin/env python

import roslib; roslib.load_manifest('rssimap')
import rospy
from geometry_msgs.msg import PoseStamped
from rssimap.msg import RssiStamped, RssiPoseStamped
from numpy import arange

pub = None
curpose = None

def rssi(data):
    msg = RssiPoseStamped(pose=curpose, rssi=data)
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)

def pose(data):
    global curpose
    curpose = data

def rssi_pose_merge():
    global pub
    rospy.init_node('rssi_pose')
    pub=rospy.Publisher('rssi_pose', RssiPoseStamped)
    rospy.Subscriber("rssi", RssiStamped, rssi)
    rospy.Subscriber("pose", PoseStamped, pose)
    rospy.spin()

if __name__ == '__main__':
    try: rssi_pose_merge()
    except rospy.ROSInterruptException:
        pass

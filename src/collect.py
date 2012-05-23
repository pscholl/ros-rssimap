#!/usr/bin/env python

import roslib; roslib.load_manifest('rssimap')
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class RandomCollect():
    """ Generates random rssi samples for testing.
    """
    def __init__(self):
        pass

    def __call__(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':
    try:
        RandomCollect()()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import roslib; roslib.load_manifest('rssimap')
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class UdpLiteCollect():
    """ Collects RSSI values from udplite packets.

    For the jnusb Zigbee platform local rssi valeus are put into UdpLite, we
    extract them here and publish them on rssi_collect_out.
    """
    def __init__(self):
        pass

    def __call__(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':
    try:
        UdpLiteCollect()()
    except rospy.ROSInterruptException:
        print "meh"
        pass

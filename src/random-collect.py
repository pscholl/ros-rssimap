#!/usr/bin/env python

# Randomly generate normalized RSSI samples with RATE, originating from IDS.
# Used for simulation purposes.

import roslib; roslib.load_manifest('rssimap')
import rospy
from random import choice
from std_msgs.msg import String,Float32,Time
from rssimap.msg import RssiStamped
from numpy import arange

RATE=1/200.
IDS=['a','b','c']
VALS=arange(1.,.1,-.05)

def random_collect():
    pub=rospy.Publisher('rssi_out', RssiStamped)
    rospy.init_node('rssi_random_collect')

    while not rospy.is_shutdown():
        msg=RssiStamped(id=choice(IDS), rssi=choice(VALS))
        pub.publish(msg)
        rospy.sleep(RATE)

if __name__ == '__main__':
    try:
        random_collect()
    except rospy.ROSInterruptException:
        pass

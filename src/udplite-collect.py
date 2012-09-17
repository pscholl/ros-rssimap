#!/usr/bin/env python

import roslib; roslib.load_manifest('rssimap')
import rospy
from std_msgs.msg import String,Float32,Time
from rssimap.msg import RssiStamped
from time import sleep
from socket import *

IPPROTO_UDLITE=136
PORT=2020
IP='ff02::1%eth1'

class UdpLiteCollect():
    """ Collects RSSI values from udplite packets.

    For the jnusb Zigbee platform local rssi valeus are put into UdpLite, we
    extract them here and publish them on rssi_collect_out.
    """
    def __init__(self):
        self.pub=rospy.Publisher('rssi', RssiStamped)
        rospy.init_node('rssi_udplite_collect')

        af,typ,proto,name,sa = getaddrinfo(IP,PORT,AF_INET6,SOCK_DGRAM,IPPROTO_UDLITE)[0]
        self.rxtx = socket(af,typ,proto)
        self.rxtx.bind(('',PORT))
        #self.rxtx.setsockopt(SOL_SOCKET,SO_BROADCAST,True)
        #self.rxtx.setsockopt(SOL_SOCKET,SO_REUSEADDR,True)
        self.rxtx.setblocking(0)

    def __call__(self):
        r=None
        while not rospy.is_shutdown():
            #rospy.spin()
            #socket.send("aa")

            try:
                r=self.rxtx.recvfrom(20)
                id,rssi=r[1][0],ord(r[0][-1])
                msg=RssiStamped(id=id,rssi=rssi/75.)
                self.pub.publish(msg)
            except Exception:
                pass

            rospy.sleep(1/60.)

if __name__ == '__main__':
    try:
        UdpLiteCollect()()
    except rospy.ROSInterruptException:
        print "meh"
        pass

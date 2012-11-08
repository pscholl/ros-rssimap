#!/usr/bin/env python
# collect rssi scan samples from nl80211 (linux >3) supported
# wifi cards
#
# make sure to set the CAP_NET_ADMIN flag if not running as root:
#  setcap cap_net_admin=eip wifi-collect.py
# and to the python interpreter
#  setcap cap_net_admin=eip /usr/bin/python2.7

import roslib; roslib.load_manifest('rssimap')
import rospy
from std_msgs.msg import String,Float32,Time
from rssimap.msg import RssiStamped
from net_tools import all_interfaces, if_nametoindex
from nlmessage import NL80211,NLException,OpenStruct
from random import sample
from string import printable
from time import time

def rotate(l,n):
    return l[n:] + l[:n]

old_bss = None
def publish_bss(pub,bssarr,dev):
    global old_bss

    if old_bss is None:
        old_bss = set(bssarr)
    else:
        bssarr = set(bssarr)-old_bss

    for bss in bssarr:
        if not hasattr(bss,"ie"): continue
        ssid = [ie.ssid for ie in bss.ie if hasattr(ie,"ssid") and\
                all(c in printable for c in ie.ssid)]
        ssid = ssid[0] if len(ssid)>0 else ""
        ssid += " (%s)"%(":".join(hex(ord(x))[2:] for x in bss.bssid))
        msg = RssiStamped(
                   id = ssid,
                  dev = dev,
                bssid = bss.bssid,
                 rssi = bss.signal/100.,
            frequency = bss.frequency)
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)

        if ssid=="hrz (0:27:d:8c:ea:90)":
            print dev, ssid, bss.frequency, bss.last_seen, "ms", bss.signal/100.

def get_nlmsg(nl802):
     try:
         return nl802.handle_event()
     except NLException, e:
         if e.num == -100:
             rospy.logerr("network %s is down!"%dev)
             exit(-1)
         elif e.num == -1:
             rospy.logerr("scanning not allowed (run as root or set CAP_NET_ADMIN)!")
             exit(-1)
         else:
             raise

class WifiDevice(object):
    def __init__(self, dev, nl802):
        self.idx  = if_nametoindex(dev)
        self.name = dev
        self.sock = nl802

    def trigger_scan(self):
        self.sock._if_index = self.idx

        if not hasattr(self,"spectrum"):
            self.sock.trigger_scan()
        else:
            toscan = []
            if len(self.dead)>2 : toscan = list(self.live) + sample(self.dead,2) + [2437]
            else:                 toscan = self.live
            #print "trigger on", toscan, set(toscan)
            toscan = list(set(toscan))
            self.sock.trigger_scan(toscan)

    def checkmsg(self,nlmsg):
        if not hasattr(nlmsg,"ifindex"):
            print "ignoring non-addressed message"
            return None
        elif nlmsg.ifindex != self.idx:
            return None

        # make sure we talk to the right device
        self.sock._if_index = self.idx

        if not hasattr(self,"spectrum") and nlmsg.type=="trigger":
            self.spectrum = nlmsg.freqs
            self.live = []
            self.dead = list(self.spectrum)

        if nlmsg.type=="scan" and not hasattr(nlmsg,"bss"):
            # scan finished message
            self.sock.get_scan_results()
        elif nlmsg.type=="scan":
            self.live = set([bss.frequency for bss in nlmsg.bss if bss.frequency in self.spectrum])
            self.dead = set([freq for freq in self.spectrum if not freq in self.live])
            bss       = [bss for bss in nlmsg.bss if bss.frequency in self.spectrum]
            self.trigger_scan()
            return bss

        return None


def wifi_collect():
    pub=rospy.Publisher('rssi', RssiStamped)
    rospy.init_node('rssi_wifi_collect')

    # init
    interfaces = [NL80211(if_name) for if_name in all_interfaces() if if_name!='wlan0']
    interfaces = [nl for nl in interfaces if nl.is_active()]
    interfaces = [nl._device for nl in interfaces]

    if len(interfaces) == 0:
        rospy.logerr("no wifi interfaces found, are they 'up'?")
        exit(-1)

    # main socket
    nl802 = NL80211(interfaces[0])
    rospy.loginfo("collecting RSSI on device(s) '%s'"%(", ".join(interfaces)))

    # create the devices
    interfaces = [WifiDevice(iface, nl802) for iface in interfaces]
    for iface in interfaces:
        iface.trigger_scan()

    timestamp = 0.
    while not rospy.is_shutdown():
        try: nlmsg = get_nlmsg(nl802)
        except NLException, e:
            if e.num==-22 or e.num==-16 : # EINVAL -> frequency not supported, EBUSY
                print e.num
                continue
            else:
                raise

        if nlmsg is None or\
           nlmsg.type=="none" or\
           nlmsg.type=="empty":
               continue

        for iface in interfaces:
            bss = iface.checkmsg(nlmsg)
            if bss is not None: publish_bss(pub,bss,iface.name)

        if time() - timestamp > 5. :
            for iface in interfaces:
                iface.trigger_scan()

            timestamp = time()

if __name__ == '__main__':
    try:
        wifi_collect()
    except rospy.ROSInterruptException:
        pass

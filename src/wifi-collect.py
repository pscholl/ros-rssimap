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
from random import choice
from std_msgs.msg import String,Float32,Time
from rssimap.msg import RssiStamped
from nlmessage import NL80211,NLException,OpenStruct
from random import sample
from string import printable

RATE=0.5

def publish_bss(pub,bssarr):
    for bss in bssarr:
        ssid = [ie.ssid for ie in bss.ie if hasattr(ie,"ssid") and\
                all(c in printable for c in ie.ssid)]
        ssid = ssid[0] if len(ssid)>0 else ""
        ssid += " (%s)"%(":".join(hex(x)[2:] for x in bss.bssid))
        msg = RssiStamped(
                   id = ssid,
                bssid = bss.bssid,
                 rssi = bss.signal/100.0,
            frequency = bss.frequency)
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)

def get_nlmsg(nl802):
     try: return nl802.handle_event()
     except NLException, e:
         if e.num == -100:
             rospy.logerr("network %s is down!"%dev)
             exit(-1)
         elif e.num == -16: # EBUSY
             return OpenStruct(type="none")
         elif e.num == -1:
             rospy.logerr("scanning not allowed (run as root or set CAP_NET_ADMIN)!")
             exit(-1)
         else:
             raise

def wifi_collect():
    pub=rospy.Publisher('rssi', RssiStamped)
    rospy.init_node('rssi_wifi_collect')

    pname = rospy.search_param('scan_device')
    dev = rospy.get_param(pname, 'wlan0')
    if type(dev)==dict: dev = 'wlan0'
    rospy.loginfo("collecting RSSI on device '%s'"%dev)

    nl802 = NL80211(dev)
    nl802.trigger_scan()
    toscan,livefreqs,supportfreqs = None,None,None
    # this is our initialization, wait until the first completed scan
    while not rospy.is_shutdown():
        while supportfreqs is None or livefreqs is None:
            nlmsg = get_nlmsg(nl802)
            if hasattr(nlmsg,"bss"):
                publish_bss(pub,nlmsg.bss)
                livefreqs = set([bss.frequency for bss in nlmsg.bss])
            elif nlmsg.type=="scan":
            # this elif on pupose since a "scan" event without the bss
            # attribute means scan just finished
                supportfreqs = set(nlmsg.freqs[1::2])
                nl802.get_scan_results()
            rospy.sleep(RATE)
        break

    # now let's just scan each frequency seperately
    while not rospy.is_shutdown():
        deadfreqs = supportfreqs - livefreqs
        k = min(len(deadfreqs),2)
        toscan = list(livefreqs) + list(sample(deadfreqs,k))

        for freq in toscan:
            nl802.trigger_scan([freq])
            scan_published = False

            while not scan_published:
                nlmsg = get_nlmsg(nl802)
                if nlmsg is None:
                    continue

                if hasattr(nlmsg,"bss"):
                    publish_bss(pub,[bss for bss in nlmsg.bss if bss.frequency==freq])
                    scan_published = True
                elif nlmsg.type=="scan":
                # this elif on pupose since a "scan" event without the bss
                # attribute means scan just finished
                    nl802.get_scan_results()

            #rospy.sleep(RATE)
        #rospy.sleep(RATE)


if __name__ == '__main__':
    try:
        wifi_collect()
    except rospy.ROSInterruptException:
        pass

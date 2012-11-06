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
from string import printable
from time import time

def rotate(l,n):
    return l[n:] + l[:n]

def publish_bss(pub,bssarr,dev):
    for bss in bssarr:
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

def wifi_collect():
    pub=rospy.Publisher('rssi', RssiStamped)
    rospy.init_node('rssi_wifi_collect')

    # init
    interfaces = [NL80211(if_name) for if_name in all_interfaces()]
    interfaces = [nl for nl in interfaces if nl.is_active()]
    interfaces = [nl._device for nl in interfaces]

    if len(interfaces) == 0:
        rospy.logerr("no wifi interfaces found, are they 'up'?")
        exit(-1)

    # main socket
    nl802 = NL80211(interfaces[0])
    rospy.loginfo("collecting RSSI on device(s) '%s'"%(", ".join(interfaces)))

    # use the full list of all 802.11 frequencies and remove on error
    # of unsupported frequency of device
    #freqs = [2412, 2417, 2422, 2427, 2432, 2437, 2442, 2447, 2452, 2457, 2462, 2467, 2472, 5180, 5200, 5220, 5240, 5260, 5280, 5300, 5320, 5500, 5520, 5540, 5560, 5580, 5600, 5620, 5640, 5660, 5680, 5700, 5745, 5765, 5785, 5805, 5825]
    freqs = []
    spectra = dict([(if_nametoindex(iface),list(freqs)) for iface in interfaces])

    # spread the spectra evenly over all devices
    step = int(len(freqs)/len(interfaces))
    for k,i in zip(spectra.keys(), range(len(interfaces))):
        spectra[k] = rotate(freqs,step*i)

    # trigger a scan on the whole spectrum of each card
    # afterwards just trigger a scan wheneve a new result is in.
    for k in spectra:
        nl802._if_index = k
        nl802.trigger_scan()

    while not rospy.is_shutdown():
        try: nlmsg = get_nlmsg(nl802)
        except NLException, e:
            if e.num==-22: # EINVAL -> frequency not supported
                # unsupported freq -> remove from scan spectrum
                del spectra[e.devid][0]

                nl802._if_index = nlmsg.ifindex
                nl802.trigger_scan(spectra[nlmsg.ifindex][0])
                rospy.loginfo("spectrum on %d has %d elements: %s",
                        nlmsg.ifindex, len(spectra[nlmsg.ifindex]),
                        str(spectra[nlmsg.ifindex]))
                continue
            if e.num==-16: # EBUSY
                continue
            else:
                raise

        if nlmsg is None or\
           nlmsg.type=="none" or\
           nlmsg.type=="empty":
               continue

        if nlmsg.type=="trigger" and spectra[nlmsg.ifindex]==[]:
            spectra[nlmsg.ifindex] = nlmsg.freqs

        if nlmsg.type=="scan" and hasattr(nlmsg,"bss"):
            freq = spectra[nlmsg.ifindex][0]
            publish_bss(pub,[bss for bss in nlmsg.bss if bss.frequency==freq], str(nlmsg.ifindex))
            spectra[nlmsg.ifindex] = rotate(spectra[nlmsg.ifindex],1)

            nl802._if_index = nlmsg.ifindex
            nl802.trigger_scan(spectra[nlmsg.ifindex][0])
            #rospy.loginfo("triggger scan on %d freq %d", nlmsg.ifindex, spectra[nlmsg.ifindex][0])
        elif nlmsg.type=="scan":
            # this elif on pupose since a "scan" event without the bss
            # attribute means scan just finished
            nl802._if_index = nlmsg.ifindex
            #rospy.loginfo("scan finished on %d", nlmsg.ifindex)
            nl802.get_scan_results()

            nl802._if_index = nlmsg.ifindex
            nl802.trigger_scan(spectra[nlmsg.ifindex][0])
            #rospy.loginfo("trigger scan on %d freq %d", nlmsg.ifindex, spectra[nlmsg.ifindex][0])

        i+=1

if __name__ == '__main__':
    try:
        wifi_collect()
    except rospy.ROSInterruptException:
        pass

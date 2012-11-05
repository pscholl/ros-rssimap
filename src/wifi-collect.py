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
         elif e.num == -16: # EBUSY
             return OpenStruct(type="busy")
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
    freqs   = [2412, 2417, 2422, 2427, 2432, 2437, 2442, 2447, 2452, 2457, 2462, 2467, 2472, 5180, 5200, 5220, 5240, 5260, 5280, 5300, 5320, 5500, 5520, 5540, 5560, 5580, 5600, 5620, 5640, 5660, 5680, 5700, 5745, 5765, 5785, 5805, 5825]
    spectra = dict([(if_nametoindex(iface),list(freqs)) for iface in interfaces])

    # spread the spectra evenly over all devices
    step = int(len(freqs)/len(interfaces))
    for k,i in zip(spectra.keys(), range(len(interfaces))):
        spectra[k] = rotate(freqs,step*i)

    # it seems that sometimes (probably when this program stop incorrectly) the output
    # gets clogged 
    for k in spectra:
        nl802._if_index=k
        nl802.get_scan_results()

    i=0
    while not rospy.is_shutdown():
        for (devidx,spectrum) in spectra.items():
            nl802._if_index = devidx
            freq = spectrum[i%len(spectrum)]

            nl802.trigger_scan(freq);
            rospy.loginfo("trigger scan on %d at %d"%(devidx,freq))

        scans_published = 0
        while scans_published < len(spectra):
            try: nlmsg = get_nlmsg(nl802)
            except NLException, e:
                if e.num==-22: # EINVAL -> frequency not supported
                    scans_published += 1
                    continue
                    # silently ignore this, since I can't find a way to
                    # find which device has caused this error
                else:
                    raise

            if nlmsg is None or\
               nlmsg.type=="none" or\
               nlmsg.type=="busy" or\
               nlmsg.type=="trigger":
                   continue

            if hasattr(nlmsg,"bss"):
                publish_bss(pub,[bss for bss in nlmsg.bss if bss.frequency==freq], str(nlmsg.ifindex))
                scans_published += 1
            elif nlmsg.type=="scan":
            # this elif on pupose since a "scan" event without the bss
            # attribute means scan just finished
                #print "get_scan", nlmsg.ifindex, nl802._if_index
                nl802._if_index = nlmsg.ifindex
                nl802.get_scan_results()
        i+=1

if __name__ == '__main__':
    try:
        wifi_collect()
    except rospy.ROSInterruptException:
        pass

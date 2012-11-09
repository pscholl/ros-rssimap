#!/usr/bin/env python
# collect rssi scan samples from nl80211 (linux >3) supported
# wifi cards
#
# make sure to set the CAP_NET_ADMIN flag if not running as root:
#  setcap cap_net_admin=eip wifi-collect.py
# and to the python interpreter
#  setcap cap_net_admin=eip /usr/bin/python2.7

import roslib; roslib.load_manifest('rssimap')
import rospy,shlex
from std_msgs.msg import String,Float32,Time
from rssimap.msg import RssiStamped
from net_tools import all_interfaces, if_nametoindex
from select import select
from subprocess import Popen,PIPE
import re

def publish(pub,freq,rssi,bssid,ssid,dev='wlan0'):
    msg = RssiStamped(
               id = "%s (%s)"%(ssid,bssid),
              dev = dev,
            bssid = [int(x,16) for x in bssid.split(':')],
             rssi = float(rssi),
        frequency = float(freq))
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    print "pub"

if __name__ == '__main__':
    pub=rospy.Publisher('rssi', RssiStamped)
    rospy.init_node('rssi_wifi_tap')

    # init
    interfaces = [if_name for if_name in all_interfaces() 
            if if_name[:4]=="wlan"]

    if len(interfaces) == 0:
        rospy.logerr("no wifi interfaces found, are they 'up'?")
        exit(-1)

    rospy.loginfo("collecting rssi on %s",str(interfaces))

    # communication with tcpdump
    beacon_parser  = re.compile('.* (\d+) MHz.* (-\d+)dB .* BSSID:([0-9A-Fa-f][0-9A-Fa-f]:[0-9A-Fa-f][0-9A-Fa-f]:[0-9A-Fa-f][0-9A-Fa-f]:[0-9A-Fa-f][0-9A-Fa-f]:[0-9A-Fa-f][0-9A-Fa-f]:[0-9A-Fa-f][0-9A-Fa-f]).* Beacon \((\S*)\) .*')
    rts_pattern    = '.* (\d+) MHz.* (-\d+)dB .* TA:(%s).*'
    cmdlines = [shlex.split("/usr/sbin/tcpdump -eIi %s"%(iface))
                for iface in interfaces]
    channels = [Popen(cmd,bufsize=4096,shell=False,stdout=PIPE).stdout
                for cmd in cmdlines]
    interfaces = dict([(f.fileno(),if_name) for (f,if_name) in zip(channels,interfaces)])
    channels = dict([(f.fileno(),f) for f in channels])

    # we also keep a set of seen beacons and add them to the list to also
    # capture their RTS packets, further increasing the sampling rate
    bssids = set()
    rts_parsers = list()

    while not rospy.is_shutdown():
        r,w,x = select(channels.keys(),[],[],.5)
        readable = [(channels[fid],interfaces[fid]) for fid in r]

        for (f,dev) in readable:
            line = f.readline()
            print "abc", line
            m = beacon_parser.match(line)
            if m is not None:
                publish(pub,*m.groups(),dev=dev)

                bssid = m.groups()[2]
                if not bssid in bssids:
                    bssids.add(bssid)
                    rts_parsers.append(re.compile(rts_pattern%bssid))

            for p in rts_parsers:
                m = p.match(line)
                if m is None: continue
                publish(pub,*m.groups(),ssid="bloierg",dev=dev)



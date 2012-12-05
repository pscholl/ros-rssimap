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
from sys import exit
import re

msg = RssiStamped()

def publish(pub,freq,rssi,bssid,ssid,dev='wlan0'):
    msg.id = "%s (%s)"%(ssid,bssid)
    msg.dev = dev
    msg.bssid = [int(x,16) for x in bssid.split(':')]
    msg.rssi = float(rssi)
    msg.frequency = float(freq)
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)

if __name__ == '__main__':
    pub=rospy.Publisher('rssi', RssiStamped)
    rospy.init_node('rssi_wifi_tap')

    # init
    interfaces = [if_name for if_name in all_interfaces()
            if if_name[:4]=="wlan"]# and if_name!="wlan2"]

    if len(interfaces) == 0:
        rospy.logerr("no wifi interfaces found, are they 'up'?")
        exit(-1)

    rospy.loginfo("collecting rssi on %s",str(interfaces))

    # communication with tcpdump
    bcn_pattern = '.* (\d+) MHz.* (-\d+)dB .* BSSID:([0-9A-Fa-f][0-9A-Fa-f]:[0-9A-Fa-f][0-9A-Fa-f]:[0-9A-Fa-f][0-9A-Fa-f]:[0-9A-Fa-f][0-9A-Fa-f]:[0-9A-Fa-f][0-9A-Fa-f]:[0-9A-Fa-f][0-9A-Fa-f]).* Beacon \((\S*)\) .*'
    rts_pattern = '.* (\d+) MHz.* (-\d+)dB .* TA:(%s).*'
    cmdlines = [shlex.split("/usr/sbin/tcpdump -eIi %s"%(iface))
                for iface in interfaces if iface!="wlan2"]
    popens = [Popen(cmd,bufsize=8192,shell=False,stdout=PIPE) for cmd in cmdlines]
    channels = [p.stdout for p in popens]
    interfaces = dict([(f.fileno(),if_name) for (f,if_name) in zip(channels,interfaces)])
    popens     = dict([(f.fileno(),p) for (f,p) in zip(channels,popens)])
    channels   = dict([(f.fileno(),f) for f in channels])

    # we also keep a set of seen beacons and add them to the list to also
    # capture their RTS packets, further increasing the sampling rate
    parser = re.compile(bcn_pattern)
    bssids = set()

    while not rospy.is_shutdown():
        r,w,x = select(channels.keys(),[],[],.5)

        for fid in r:
            f,dev = channels[fid],interfaces[fid]
            line = f.readline()
            m = parser.match(line)
            if m is not None:
                publish(pub,*m.groups()[:4],dev=dev)

                #bssid = m.groups()[2]
                #if not bssid in bssids:
                #    bssids.add(bssid)
                #    parser = re.compile("|".join([bcn_pattern]+[rts_pattern%bssid for bssid in bssids]))

            else: # check if the tcpdump process died
                if popens[fid].poll() is not None:
                    rospy.logerr("shutting down interface %s"%(dev))
                    del channels[fid],interfaces[fid]

                    if len(interfaces)==0:
                        rospy.logerr("no more devices to scan on, shutting down!")
                        exit(-1)


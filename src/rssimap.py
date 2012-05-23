#!/usr/bin/env python

# Implements the off-line of fingerprinting RSSI mapping algorithms, i.e.
# it collects rssi-measurements (/rssi_out topic) and pose estimations 
# (/slam_out). And combines the position and rssi measurements. And publishes
# this data again.

import roslib; roslib.load_manifest('rssimap')
import rospy, numpy as np
from random import random
from std_msgs.msg import String,Float32,Time,ColorRGBA
from rssimap.msg import RssiStamped,RssiUniqueStamped
from geometry_msgs.msg import PoseStamped,Point
from visualization_msgs.msg import Marker
from colorschemes import torgba
from time import time

class RadioMap():
    """ stores rssi value at position in a non-uniform grid with the given
    resolution. The values are compressed, since the number of unique RSSI
    values is limited, we only store the number of total occurences of a
    specific value.

    XXX: We probably need to implement this with an rtree to allow for range
    queries.
    """
    def __init__(self,resolution=.5,cb=None):
        self.grid={}
        self.ids=[]
        self.cb=cb

        def tores(n):
            rest = n%resolution
            if rest > resolution/2.:
                return n+(resolution-rest)
            else:
                return n-rest

        self.tores=tores
        self.max_x=self.max_y=-5000
        self.min_y=self.min_x=5000
        self.t=0
        self.tt=0

    def update_pose(self, msg):
        x,y=msg.pose.position.x,msg.pose.position.y

        self.x=self.tores(x)
        self.y=self.tores(y)

        self.max_x = max(self.max_x, self.x)
        self.min_x = min(self.min_x, self.x)
        self.max_y = max(self.max_y, self.y)
        self.min_y = min(self.min_y, self.y)

        #for radiomap in grid.items():
        #    radiomap[(x,y)]

    def update_rssi(self, rssi):
        #t=time()
        if not hasattr(self,"x"):
            return

        x,y,grid=self.x,self.y,self.grid

        # make sure to create a dict for each id
        if not grid.has_key(rssi.id):
            grid[rssi.id] = {}
            self.ids.append(rssi.id)
        radiomap = grid[rssi.id]

        # make sure that there is a dict at x,y
        if not radiomap.has_key((x,y)):
            radiomap[(x,y)] = {}

        # and one for each rssi-value
        if not radiomap[(x,y)].has_key(rssi.rssi):
            radiomap[(x,y)][rssi.rssi] = 0
        radiomap[(x,y)][rssi.rssi] += 1

        #rospy.loginfo(rospy.get_name()+
        #        " added at %f %f %s %f"%(x,y,rssi.id,rssi.rssi))

        if self.cb: self.cb(self,x,y)

        #print("rssi")
        #self.tt+=time()-t

        print "rssicb: ",self.tt,self.t,self.tt/self.t
        self.t=self.tt=0

    def vectorize(self,x,y, fill_value=0.):
        """ return the sorted rssi measurement vector at pos x,y, filled
        with fill_value where no measurement is available.
        """
        #t=time()

        # this dict maps from rssi-value -> number of occurences, rssi_mean calculates
        # the mean of those observations...
        rssi_mean=lambda d: np.sum([rssi*occur for rssi,occur in d.items()])/np.sum(d.values())

        # all measurements we have
        v = [rssi_mean(self.grid[id][(x,y)]) if self.grid[id].has_key((x,y)) else 0 for id in sorted(self.ids)]

        # and finally convert to numpy array
        #self.t+=time()-t
        return np.array(v)

class UniqueMap():
    def __init__(self):
        self.grid = {}
        self.colors = {}

        self.pub_unique = rospy.Publisher('rssimap_uniquess_out', RssiUniqueStamped)
        self.pub_rviz   = rospy.Publisher('visualization_marker', Marker)

        marker = Marker()
        marker.header.frame_id='/map'
        marker.ns='rssimap'
        marker.id=0
        marker.type=Marker.CUBE_LIST
        marker.action=Marker.ADD
        marker.scale.x=.7 # XXX: this should reflect the resolution chosen in RSSIMAP
        marker.scale.y=.7
        marker.color.a=.5
        #marker.lifetime.secs=2000
        self.marker=marker

    def __call__(self,rmap,x,y):
        tt=time()
        grid,colors,marker = self.grid,self.colors,self.marker

        # check if there is already something on the distance grid
        try: grid[(x,y)]
        except KeyError:
            grid[(x,y)] = 1. # XXX: bad assumption, check this!
            colors[Point(x,y,0)] = ColorRGBA(*torgba(grid[(x,y)]))

        # this is the vector we are comparing
        vector = rmap.vectorize(x,y)**2

        # calculate the absolute distance to all other measurments vectors,
        # and update the list if there is a new minimum.
        update=[]
        fugrid=zip(np.arange(rmap.min_x,rmap.max_x),np.arange(rmap.min_y,rmap.max_y))
        for (i,j) in [t for t in fugrid if t!=(x,y)]:
            other = rmap.vectorize(i,j)**2
            if np.sum(other)==0: # ignore zero-vectors
                continue

            distance = np.sqrt(np.sum(vector-other))

            if grid[(x,y)] > distance:
                grid[(x,y)] = distance
                colors[Point(x=x,y=y)] = ColorRGBA(*torgba(distance,scheme='fire'))
                update.append((x,y))
                # also update the vector we compared to
                grid[(i,j)] = distance
                colors[Point(x=i,y=j)] = ColorRGBA(*torgba(distance,scheme='fire'))
                update.append((i,j))

        # check if there was an update, if so publish
        for (x,y) in update:
            self.pub_unique.publish(RssiUniqueStamped(x=x,y=y,min_distance=grid[(x,y)]))

            marker.header.stamp=rospy.get_rostime()
            marker.points=colors.keys()
            marker.colors=colors.values()
            self.pub_rviz.publish(marker)

            rospy.loginfo(rospy.get_name()+
                " recalc at %f %f to %f"%(x,y,grid[(x,y)]))


if __name__ == '__main__':
    try:
        r=RadioMap(cb=UniqueMap())
        rospy.init_node('rssimap')
        rospy.Subscriber('/slam_out_pose', PoseStamped, r.update_pose)
        rospy.Subscriber('/rssi_out', RssiStamped, r.update_rssi)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

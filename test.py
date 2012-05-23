#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import rospy
import math

topic = 'visualization_marker'
publisher = rospy.Publisher(topic, Marker)
rospy.init_node('register')

count = 0
MARKERS_MAX = 100

while not rospy.is_shutdown():

   marker = Marker()
   marker.header.frame_id = "/map"
   marker.type = marker.CUBE_LIST
   marker.ns = 'safsldf'
   marker.action = marker.MODIFY
   marker.scale.x = .2
   marker.scale.y = 20
   marker.scale.z = 0
   marker.color.a = 0.5
   #marker.color.r = 1.0
   #marker.color.g = 1.0
   #marker.color.b = 0.0
   #marker.pose.orientation.w = 1.0
   #marker.pose.position.x = 1.0
   #marker.pose.position.y = 1.0
   #marker.pose.position.z = 1.0
   marker.points=[Point(x,1,5) for x in range(2)]
   marker.colors=[ColorRGBA(1,0,1,0)]*2

   # Publish the MarkerArray
   publisher.publish(marker)

   count += 1

   rospy.sleep(0.01)


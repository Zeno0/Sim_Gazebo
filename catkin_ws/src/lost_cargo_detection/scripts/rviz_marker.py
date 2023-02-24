#! /usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker

rospy.init_node('rviz_marker')

marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)



while not rospy.is_shutdown():
  
  for i in range(20):
    rospy.sleep(1)
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 2
    marker.id = 0
    # Set the scale of the marker
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    # Set the color
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    # # Set the pose of the marker
    # marker.pose.position.x = 2.7570763
    # marker.pose.position.y = 10.88049
    # marker.pose.position.z = 0
    # marker.pose.orientation.x = 0.0
    # marker.pose.orientation.y = 0.0
    # marker.pose.orientation.z = 0.0
    # marker.pose.orientation.w = 1.0
    # Set the pose of the marker
    marker.pose.position.x = i
    marker.pose.position.y = i
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker_pub.publish(marker)
  
  rospy.rostime.wallsleep(1.0)
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud 

def callback(data):
    data = PointCloud()
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(data.header.stamp.secs))
    


    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('lost_cargo_detection_node', anonymous=True)

    rospy.Subscriber("/prius/center_laser/scan", PointCloud, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
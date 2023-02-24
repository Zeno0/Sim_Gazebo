import rospy
from sensor_msgs.msg import PointCloud2 
import pcl
from pcl_helper import ros_to_pcl ,pcl_to_ros , my_pcl_to_ros
from open3d import *
import numpy as np
import time
# from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs

class simple_class():


    def __init__(self):
        super().__init__('pcd_subsriber_node')

        ## This is for visualization of the received point cloud.
        self.sub = rospy.Subscriber("/prius/center_laser/scan/pointcloud2", PointCloud2, self.listener_callback)
        self.vis = open3d.visualization.Visualizer()
        self.vis.create_window()
        self.o3d_pcd = open3d.geometry.PointCloud()

        # Set up a subscription to the 'pcd' topic with a callback to the 
        # function `listener_callback`
        # self.pcd_subscriber = self.create_subscription(
        #     sensor_msgs.PointCloud2,    # Msg type
        #     'pcd',                      # topic
        #     self.listener_callback,      # Function to call
        #     10                          # QoS
        # )

                
    def listener_callback(self, msg):
        # Here we convert the 'msg', which is of the type PointCloud2.
        # I ported the function read_points2 from 
        # the ROS1 package. 
        # https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/src/sensor_msgs/point_cloud2.py

        # pcd_as_numpy_array = np.array(list(read_points(msg)))
        pcd_as_numpy_array = ros_to_pcl(msg)
        # The rest here is for visualization.
        self.vis.remove_geometry(self.o3d_pcd)
        self.o3d_pcd = open3d.geometry.PointCloud(
                            open3d.utility.Vector3dVector(pcd_as_numpy_array))

        self.vis.add_geometry(self.o3d_pcd)

        
        
        self.vis.poll_events()
        self.vis.update_renderer()


def pointcloud_callback(pcl_msg):
    # Convert ROS msg to PCL data
    vis = open3d.visualization.Visualizer()
    vis.create_window()
    o3d_pcd = open3d.geometry.PointCloud()
    cloud = ros_to_pcl( pcl_msg )
    cloud = np.asarray(cloud)
    cloud_pcd = cloud[:, :3]

    o3d_pcd = open3d.geometry.PointCloud(
                            open3d.utility.Vector3dVector(cloud_pcd))
    vis.add_geometry(o3d_pcd)
    vis.poll_events()
    vis.update_rendererPCDListener()
    #output_pointcloud = PointCloud2()

def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('simple_class', anonymous=True)

    # rospy.Subscriber("/prius/center_laser/scan/pointcloud2", PointCloud2, pointcloud_callback)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()

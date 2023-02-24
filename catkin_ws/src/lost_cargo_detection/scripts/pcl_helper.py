#!/usr/bin/env python3

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# 

# Import modules
import rospy
import pcl
import numpy as np
import ctypes
import struct
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from random import randint
from sklearn.neighbors import NearestNeighbors
from sklearn.cluster import DBSCAN, AgglomerativeClustering, KMeans
from sklearn.preprocessing import StandardScaler


def dbscan(X, eps, min_samples):
    ss = StandardScaler()
    X = ss.fit_transform(X)
    db = DBSCAN(eps=eps, min_samples=min_samples)
    db.fit(X)
    y_pred = db.fit_predict(X)
    print(len(db.labels_))
    # plt.scatter(X[:,0], X[:,1],c=y_pred, cmap='Paired')
    # plt.title("DBSCAN")
    # plt.show()
    # print(db)
    # print(X)
    # print(X.shape)
    print(y_pred)
    print(y_pred.shape)
    print("Done dbscan")
    return db,X
    



    # method 2
    # dbscan = DBSCAN(eps = .29, min_samples = 4).fit(X) # fitting the model
    # labels = dbscan.labels_
    # print(X.shape)
    # plt.scatter(X[:, 0], X[:,1], c = labels, cmap= "plasma") # plotting the clusters
    # plt.xlabel("x") # X-axis label
    # plt.ylabel("y") # Y-axis label
    # plt.show()
    # plt.show(block=False)
    # plt.pause(3)
    # plt.close()


def random_color_gen():
    """ Generates a random color
    
        Args: None
        
        Returns: 
            list: 3 elements, R, G, and B
    """
    r = randint(0, 255)
    g = randint(0, 255)
    b = randint(0, 255)
    return [r, g, b]


def ros_to_pcl(ros_cloud):
    """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB
    
        Args:
            ros_cloud (PointCloud2): ROS PointCloud2 message
            
        Returns:
            pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
    """
    points_list = []

    for data in pc2.read_points(ros_cloud, skip_nans=True):
        points_list.append([data[0], data[1], data[2], data[3]])

    # pcl.PointCloud()
    pcl_data = pcl.PointCloud_PointXYZRGB()
    
    # pcl.
    pcl_data.from_list(points_list)

    return pcl_data


def ros_to_pcl1(ros_cloud):
    """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB
    
        Args:
            ros_cloud (PointCloud2): ROS PointCloud2 message
            
        Returns:
            pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
    """
    
    points_list = []

    for data in pc2.read_points(ros_cloud, skip_nans=True):
        points_list.append([data[0], data[1], data[2]])

    pcl_data = pcl.PointCloud_PointXYZRGB()
    pcl_data.from_list(points_list)

    return pcl_data


def pcl_to_ros(pcl_array):
    """ Converts a pcl PointXYZRGB to a ROS PointCloud2 message
    
        Args:
            pcl_array (PointCloud_PointXYZRGB): A PCL XYZRGB point cloud
            
        Returns:
            PointCloud2: A ROS point cloud
    """
    ros_msg = PointCloud2()

    ros_msg.header.stamp = rospy.Time.now()
    ros_msg.header.frame_id = "world"

    ros_msg.height = 1
    ros_msg.width = pcl_array.size

    ros_msg.fields.append(PointField(
                            name="x",
                            offset=0,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="y",
                            offset=4,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="z",
                            offset=8,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="rgb",
                            offset=16,
                            datatype=PointField.FLOAT32, count=1))

    ros_msg.is_bigendian = False
    ros_msg.point_step = 32
    ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height
    ros_msg.is_dense = False
    buffer = []

    for data in pcl_array:
        s = struct.pack('>f', data[3])
        i = struct.unpack('>l', s)[0]
        pack = ctypes.c_uint32(i).value

        r = (pack & 0x00FF0000) >> 16
        g = (pack & 0x0000FF00) >> 8
        b = (pack & 0x000000FF)

        buffer.append(struct.pack('ffffBBBBIII', data[0], data[1], data[2], 1.0, b, g, r, 0, 0, 0, 0))
    
    ros_msg.data = "".join(buffer)

    return ros_msg


def XYZRGB_to_XYZ(XYZRGB_cloud):
    """ Converts a PCL XYZRGB point cloud to an XYZ point cloud (removes color info)
    
        Args:
            XYZRGB_cloud (PointCloud_PointXYZRGB): A PCL XYZRGB point cloud
            
        Returns:
            PointCloud_PointXYZ: A PCL XYZ point cloud
    """
    XYZ_cloud = pcl.PointCloud()
    points_list = []

    for data in XYZRGB_cloud:
        points_list.append([data[0], data[1], data[2]])

    XYZ_cloud.from_list(points_list)
    return XYZ_cloud


def XYZ_to_XYZRGB(XYZ_cloud, color):
    """ Converts a PCL XYZ point cloud to a PCL XYZRGB point cloud
    
        All returned points in the XYZRGB cloud will be the color indicated
        by the color parameter.
    
        Args:
            XYZ_cloud (PointCloud_XYZ): A PCL XYZ point cloud
            color (list): 3-element list of integers [0-255,0-255,0-255]
            
        Returns:
            PointCloud_PointXYZRGB: A PCL XYZRGB point cloud
    """
    XYZRGB_cloud = pcl.PointCloud_PointXYZRGB()
    points_list = []

    float_rgb = rgb_to_float(color)

    for data in XYZ_cloud:
        points_list.append([data[0], data[1], data[2], float_rgb])

    XYZRGB_cloud.from_list(points_list)
    return XYZRGB_cloud


def rgb_to_float(color):
    """ Converts an RGB list to the packed float format used by PCL
    
        From the PCL docs:
        "Due to historical reasons (PCL was first developed as a ROS package),
         the RGB information is packed into an integer and casted to a float"
    
        Args:
            color (list): 3-element list of integers [0-255,0-255,0-255]
            
        Returns:
            float_rgb: RGB value packed as a float
    """
    hex_r = (0xff & color[0]) << 16
    hex_g = (0xff & color[1]) << 8
    hex_b = (0xff & color[2])

    hex_rgb = hex_r | hex_g | hex_b

    float_rgb = struct.unpack('f', struct.pack('i', hex_rgb))[0]

    return float_rgb


def float_to_rgb(float_rgb):
    """ Converts a packed float RGB format to an RGB list    
        
        Args:
            float_rgb: RGB value packed as a float
            
        Returns:
            color (list): 3-element list of integers [0-255,0-255,0-255]
    """
    s = struct.pack('>f', float_rgb)
    i = struct.unpack('>l', s)[0]
    pack = ctypes.c_uint32(i).value
			
    r = (pack & 0x00FF0000) >> 16
    g = (pack & 0x0000FF00) >> 8
    b = (pack & 0x000000FF)
			
    color = [r,g,b]
			
    return color

            
def get_color_list(cluster_count):
    """ Returns a list of randomized colors
    
        Args:
            cluster_count (int): Number of random colors to generate
            
        Returns:
            (list): List containing 3-element color lists
    """
    if (cluster_count > len(get_color_list.color_list)):
        for i in xrange(len(get_color_list.color_list), cluster_count):
            get_color_list.color_list.append(random_color_gen())
    return get_color_list.color_list


def my_pcl_to_ros(cloud_pcl):
    """ Converts a pcl to a ROS PointCloud2 message """

    # Instantiating a PointCloud2 object.
    cloud_ros = PointCloud2()
    # Converting pcl to numpy array.
    cloud = cloud_pcl.to_array()

    # Filling the PointCLoud2 message definition #

    #header
    cloud_ros.header.stamp = rospy.Time.now()
    cloud_ros.header.frame_id = "velodyne"

    #width & height
    h, w = cloud.shape
    cloud_ros.width = w
    cloud_ros.height = h

    #fields
    cloud_ros.fields.append(PointField(
                            name="x",
                            offset=0,
                            datatype=PointField.FLOAT32, count=1))
    cloud_ros.fields.append(PointField(
                            name="y",
                            offset=4,
                            datatype=PointField.FLOAT32, count=1))
    cloud_ros.fields.append(PointField(
                            name="z",
                            offset=8,
                            datatype=PointField.FLOAT32, count=1))
    cloud_ros.fields.append(PointField(
                            name="intensity",
                            offset=12,
                            datatype=PointField.FLOAT32, count=1))

    #booleans
    cloud_ros.is_dense = False
    cloud_ros.is_bigendian = False

    #steps
    # ps = getsizeof(cloud[0,0])
    cloud_ros.point_step = 16
    cloud_ros.row_step = ps * w

    #data
    #tostring converts to bytes, name is just misleading.
    cloud_ros.data = cloud.tostring()

    return cloud_ros


def array_to_pointcloud2(cloud_arr, stamp=None, frame_id=None):
    '''Converts a numpy record array to a sensor_msgs.msg.PointCloud2.
    '''
    # make it 2d (even if height will be 1)
    cloud_arr = np.atleast_2d(cloud_arr)

    cloud_msg = PointCloud2()

    if stamp is not None:
        cloud_msg.header.stamp = stamp
    if frame_id is not None:
        cloud_msg.header.frame_id = frame_id
    cloud_msg.height = cloud_arr.shape[0]
    cloud_msg.width = cloud_arr.shape[1]
    cloud_msg.fields = dtype_to_fields(cloud_arr.dtype)
    cloud_msg.is_bigendian = False # assumption
    cloud_msg.point_step = cloud_arr.dtype.itemsize
    cloud_msg.row_step = cloud_msg.point_step*cloud_arr.shape[1]
    cloud_msg.is_dense = all([np.isfinite(cloud_arr[fname]).all() for fname in cloud_arr.dtype.names])
    cloud_msg.data = cloud_arr.tostring()
    return cloud_msg 
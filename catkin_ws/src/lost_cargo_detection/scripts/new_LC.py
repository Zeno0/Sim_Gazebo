#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import pcl
from pcl_helper import ros_to_pcl ,pcl_to_ros , my_pcl_to_ros , array_to_pointcloud2
from open3d import *
import numpy as np
import time
import sensor_msgs.msg as sensor_msgs
import open3d as o3d
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors
from sklearn.cluster import DBSCAN, AgglomerativeClustering, KMeans
from sklearn.preprocessing import StandardScaler
import seaborn as sns
from collections import Counter
import time
from open3d import *
import math
import numpy as np
import itertools
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointField

from std_msgs.msg import Header

import copy
import hdbscan
from visualization_msgs.msg import Marker , MarkerArray
from collections import Counter

# to get values in rviz for visualization purpose
pointcloud_publisher = rospy.Publisher('/output/cloud', PointCloud2, queue_size=10)
marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)
marker_array_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size = 2)

# global variables used to solve problem of matplotlib
X_coordinates = []
Y_coordinates = []
Y_predictions = None

HEADER = Header(frame_id='base_link')

FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
   
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
]


def convertCloudROSToOpend3d(ros_msg):
    field_names = [field.name for field in ros_msg.fields]
    cloud_data = list(pc2.read_points(ros_msg, skip_nans=True, field_names= field_names))
    o3d_cloud = o3d.geometry.PointCloud()
    if len(cloud_data) == 0:
        return None
    xyz = []
    time = []
    n = 0
    for data in cloud_data:
        # print(data)
        xyz.append([data[0],data[1],data[2]])
        time.append(data[3])
        n += 1
    # print(xyz)
    o3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz[1:]))
    return o3d_cloud,np.array(xyz[1:])

def pointcloud_callback(pcl_msg):
    # Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    # print(type(cloud))
    o3d_cloud, arr = convertCloudROSToOpend3d(pcl_msg)
    #p.from_array(cloud_np, dtype=np.float32)
    #p.from_array(cloud_np)
    # print(cloud_np)
    seg = cloud.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.20)
    indices, coef = seg.segment()
    # print(indices)
    # print(coef)
    # np.where((x < -1.7) & (x > -0.7))
    cloud_filt = cloud.extract(indices, negative=True)
    # print(type(cloud_filt))
    # print(len(cloud_filt))
    # seg = cloud.makesegmenter()
    # print("-"*100)
    # print("\t \t Printing np cloud ")
    # print(cloud_np)
    # print("-"*100)
    pcd = o3d_cloud
    # pcd_sel = pcd.select_by_index(np.where(points[:, 1] > 0)[0])
    cloud_np = np.asarray(cloud_filt)
    x = cloud_np[:, 0]
    y = cloud_np[:, 1]
    z = cloud_np[:, 2]
    xmin = np.where((x < -0.7) & (x > -2))
    ymin = np.where((y < 1) & (y > -7))
    # zmin = np.where(z < -1.79)
    # print(zmin)
    common = np.intersect1d(xmin, ymin)
    cloud_np[common] = 0
    # cloud_np[zmin] = 0
    publish_points(tuple(cloud_np))
    # print(cloud_np.shape)
    # way 1
    # print(cloud_np.shape)
    dbscan(input_cloud=cloud_np, eps=.65, min_samples=2)
    # way 2
    # nn = find_dbscan_clusters(cloud_np)
    # got_cluster_indices(y_got)

def publish_points(data):

    
    publisher = rospy.Publisher('/custom_point_cloud', PointCloud2, queue_size=1)
    point_cloud = pc2.create_cloud(HEADER, FIELDS, data)
    r = rospy.Rate(1)
    publisher.publish(point_cloud)

def find_dbscan_clusters(data):
    data_points=data
    epsilon=.8
    min_samples=2
    db=DBSCAN(eps=epsilon, min_samples=min_samples)
    db.fit(data_points)
    return db.labels_


def hdbscan(in_cloud):
    clusterer = hdbscan.HDBSCAN()
    clusterer.fit(in_cloud)
    y_pred = clusterer.labels_
    
    
    

def dbscan(input_cloud, eps, min_samples):
    
    # ss = StandardScaler()
    # input_cloud = ss.fit_transform(input_cloud)
    # print(input_cloud.shape)
    db = DBSCAN(eps=eps, min_samples=min_samples)
    db.fit(input_cloud)
    y_pred = db.fit_predict(input_cloud)
    # y_pred = db.labels_
    Y_predictions = y_pred
    ##########################################################################################
    b = Counter(y_pred) 
    # n has two most occured value in predicted clusters for each data point i.e < 8000 
    n = b.most_common(2)  
    print(n)
    print(n[1][0])
    X1_cluster = input_cloud[:,0][np.where(y_pred == n[0][0])]
    tobemeanx = np.mean(X1_cluster)
    tobemedianx = np.median(X1_cluster)
    Y1_cluster = input_cloud[:,1][np.where(y_pred == n[0][0])]
    tobemeany = np.mean(Y1_cluster)
    tobemediany = np.median(Y1_cluster)
    Z1_cluster = input_cloud[:,2][np.where(y_pred == n[0][0])]
    tobemeanz = np.mean(Z1_cluster)
    tobemedianz = np.median(Z1_cluster)
    ######################################################################################

    # print(y_pred.shape)
    global X_coordinates, Y_coordinates
    X_coordinates = input_cloud[:,0]
    Y_coordinates = input_cloud[:,1]

    # other_pred = find_dbscan_clusters(input_cloud)
    # c = Counter(other_pred)  
    # nn = c.most_common(2)
    # X1_cluster = input_cloud[:,0][np.where(other_pred == nn[0][0])] 
    # tobemeanx = np.mean(X1_cluster)
    # tobemedianx = np.median(X1_cluster)
    # Y1_cluster = input_cloud[:,1][np.where(other_pred == nn[0][0])]
    # tobemeany = np.mean(Y1_cluster)
    # tobemediany = np.median(Y1_cluster)
    # Z1_cluster = input_cloud[:,2][np.where(other_pred == nn[0][0])]
    # tobemeanz = np.mean(Z1_cluster)
    # tobemedianz = np.median(Z1_cluster)
    print('done \n')

    publish_marker(tobemeanx,tobemeany,tobemeanz)
    # publish_marker_array(X_coordinates=X_coordinates,Y_coordinates=Y_coordinates)
    # plt.scatter(input_cloud[:,0], input_cloud[:,1],c=y_pred, cmap='Paired')
    # plt.title("DBSCAN")
    # plt.show()
    # print("Done dbscan")
    

def publish_marker(X_coordinates , Y_coordinates, Z_coordinates):
    # print(X_coordinates,Y_coordinates)
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
    
    marker.pose.position.x = X_coordinates
    marker.pose.position.y = Y_coordinates
    marker.pose.position.z = Z_coordinates
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker_pub.publish(marker)


def publish_marker_array(X_coordinates , Y_coordinates):
    marker_array = MarkerArray()
    count = 0
    for X, Y in [(x,y) for x in X_coordinates for y in Y_coordinates]:
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 2
        marker.id = count
        count = count +1
        # Set the scale of the marker
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.10
        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.pose.position.x = X
        marker.pose.position.y = Y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker_array.markers.append(marker)
    
    marker_array_pub.publish(marker_array)
    
def hier(X, n_clusters):
    ss = StandardScaler()
    X = ss.fit_transform(X)
    hier = AgglomerativeClustering(n_clusters=n_clusters)
    y_pred = hier.fit_predict(X)
    print("Done hier")

def listener():
    rospy.init_node('lost_cargo_detection_node', anonymous=True)
    rospy.Subscriber("/prius/center_laser/scan/pointcloud2", PointCloud2, pointcloud_callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()
    r = rospy.Rate(5)

    while True:
        # print("-"*100)
        # print("\t \t PRINTING X AND Y AT LISTENER ")
        # print(X_coordinates)
        # print(Y_coordinates)
        # print("-"*100)
        # plt.scatter(X_coordinates, Y_coordinates,c=Y_predictions, cmap='Paired')
        # plt.title("Hierarchical")
        # plt.show()
        r.sleep()





if __name__ == '__main__':
    listener()
    # dbscan()
# WHEN STANDARD SCALE IS REMOVED PLOT IS NOT VISIBLE 
# VICE VERSA IS NOT TRUE
# STANDARD SCALE IS REMOVED BECAUSE AFTER SCALING POINTS ARE NOT CORRECT FOR RVIZMARKER
# inversed = scaler.inverse_transform(scaled)
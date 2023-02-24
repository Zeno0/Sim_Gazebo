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
import copy
import matplotlib
# matplotlib.use('agg')
from visualization_msgs.msg import Marker , MarkerArray
from collections import Counter

# from pcl import *

pointcloud_publisher = rospy.Publisher('/output/cloud', PointCloud2, queue_size=10)
marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)


# marker_array_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size = 2)


X_coordinates = []
Y_coordinates = []
Y_predictions = None

# pcl.


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
    # cloud = pc2_to_pcl(pcl_msg) 
    # print(cloud)
    o3d_cloud, arr = convertCloudROSToOpend3d(pcl_msg)
    cloud_np = np.asarray(cloud)
    seg = cloud.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.05)
    indices, coef = seg.segment()
    print(indices)
    print(coef)
    cloud_filt = cloud.extract(indices, negative=True)
    # segmentation
    # seg_cloud = pcl1.PointCLoud()

    # seg = seg_cloud.make_segmenter()




    # cloud_np[:, 2] =  cloud_np[:, 2] > 0.0
    # cloud_np[:, 2] =  cloud_np[:, 2] >  -0.05159756541252136
    # var = np.where(cloud_np[:, 2] > 0.0)
    # print(max(cloud_np[:, 2]))
    # cloud_np[:, 2] =  var
    # new_cloudnp = cloud_np[var]


    print("-"*100)
    print("\t \t Printing np cloud ")
    # print(z)
    # print(var)
    print(cloud_filt)
    # print(new_cloudnp.shape)
    print("-"*100)

    pcd = o3d_cloud
    print('DEbug : 3')
    # pcd_sel = pcd.select_by_index(np.where(points[:, 1] > 0)[0])
    y_got = dbscan(input_cloud=cloud_filt, eps=.8, min_samples=5)
    # eps(cloud_np)
    # nn = find_dbscan_clusters(cloud_np)
    # got_cluster_indices(y_got)



    # start_time = time.time()

def find_dbscan_clusters(data):
    data_points=data
    epsilon=.5
    min_samples=8
    db=DBSCAN(eps=epsilon, min_samples=min_samples)
    db.fit(data_points)

    return db.labels_
        

        
# toy_data=create_data()
# cluster_and_noise_labels=find_dbscan_clusters(toy_data)
# display_clusters(toy_data,cluster_and_noise_labels)
def eps(data):
    dataset = data
    neighbors = NearestNeighbors(n_neighbors=20)
    neighbors_fit = neighbors.fit(dataset)
    distances, indices = neighbors_fit.kneighbors(dataset)
    distances = np.sort(distances, axis=0)
    distances = distances[:,1]
    plt.plot(distances)

def dbscan(input_cloud, eps, min_samples):
    # print(input_cloud)
    # ss = StandardScaler()
    # input_cloud = ss.fit_transform(input_cloud)
    # print(input_cloud)
    db = DBSCAN(eps=eps, min_samples=min_samples)
    db.fit(input_cloud)
    y_pred = db.fit_predict(input_cloud)
    # y_pred = db.labels_
    Y_predictions = y_pred

    
    # print("-"*100)
    # print("\t \t FOUND 1 at index ")
    # print(np.where(y_pred == 1))
    # print("-"*100)

    # print("-"*100)
    # print("\t \t X in DBSCAN ")
    # tobemeanx = input_cloud[:,0][np.where(y_pred == 1)]
    # print(input_cloud[:,0][np.where(y_pred == 1)])
    # print(max(y_pred))
    ##########################################################################################
    b = Counter(y_pred)  
    n = b.most_common(2)  
    # print(n)
    # print(n[0][0])

    X1_cluster = input_cloud[:,0][np.where(y_pred == int(n[0][0]))]
    # # print(X1_cluster)
    tobemeanx = np.mean(X1_cluster)
    tobemedianx = np.median(X1_cluster)
    # #print(tobemeanx)
    # # print(np.mean(tobemeanx))
    # # print("-"*100)

    # # print("-"*100)
    # # print("\t \t Y in DBSCAN ")
    # tobemeany = input_cloud[:,1][np.where(y_pred == 1)]
    # # print(np.mean(tobemeany))
    # print(input_cloud[:,1][np.where(y_pred == 1)])
    Y1_cluster = input_cloud[:,1][np.where(y_pred == int(n[0][0]))]
    tobemeany = np.mean(Y1_cluster)
    tobemediany = np.median(Y1_cluster)
    # # print(Y1_cluster)
    # # print("-"*100)

    Z1_cluster = input_cloud[:,2][np.where(y_pred == int(n[0][0]))]
    tobemeanz = np.mean(Z1_cluster)
    tobemedianz = np.median(Z1_cluster)
    print(Z1_cluster)
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






    
    publish_marker(tobemeanx,tobemeany)
    # publish_marker_array(X_coordinates= X1_cluster, Y_coordinates=Y1_cluster)

    #publish_marker_array(X_coordinates=X_coordinates,Y_coordinates=Y_coordinates)
    # plt.ion()
    print('DEbug : 4')
    # plt.scatter(input_cloud[:,0], input_cloud[:,1],c=y_pred, cmap='Paired')
    # plt.title("DBSCAN")
    # plt.savefig('/home/cthalia/SHEKHAR/'+ str(frame_num) + '.png')
    # print('/home/cthalia/SHEKHAR/'+ str(frame_num) + '.png')
    print('DEbug : 5')
    plt.scatter(input_cloud[:,0], input_cloud[:,1],c=y_pred, cmap='Paired')
    plt.title("DBSCAN")
    plt.show()
    #plt.pause(90)
    #plt.close("all")
    # print("Done dbscan")
    


def publish_marker(X_coordinates , Y_coordinates):
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
    marker.pose.position.z = 1.2205082178115845
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
def hier(X, n_clusters):
    ss = StandardScaler()
    X = ss.fit_transform(X)
    hier = AgglomerativeClustering(n_clusters=n_clusters)
    y_pred = hier.fit_predict(X)
    print("Done hier")

# r = rospy.rate(5)

# while True:
#     pub.publish(data)
#     r.sleep()
#     X = ss.fit_transform(X)
#     km = KMeans(n_clusters=n_clusters)
#     km.fit(X)ospy.Subscriber("/prius/center_laser/scan/pointcloud2", PointCloud2, pointcloud_callback)
#     y_pred = km.predict(X)
#     print("Done kmeans")

def listener():
    
    rospy.init_node('lost_cargo_detection_node', anonymous=True)

    # for i in range(0,15):
    print("DEbug : 1")
    rospy.Subscriber("/prius/center_laser/scan/pointcloud2", PointCloud2, pointcloud_callback)
    print("DEbug : 2")
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()
    r = rospy.Rate(5)
        
        
        # plt.scatter(X_coordinates, Y_coordinates,c=Y_predictions, cmap='Paired')
        # plt.title("Hierarchical")
        # plt.show()
# ter(X_coordinates, Y_coordinates,c=Y_predictions, cmap='Paired')
        # plt.title("Hierarchical")
        # plt.show
    while True:
    # #     plt.title("Hierarchical")
    #     # plt.show()
    #     #plt.ion()
        # rospy.Subscriber("/prius/center_laser/scan/pointcloud2", PointCloud2, pointcloud_callback)
    #     # print("-"*100)
    #     # print("\t \t PRINTING X AND Y AT LISTENER ")
    #     # print(X_coordinates)
    #     # print(Y_coordinates)
    #     # print("-"*100)
        plt.scatter(X_coordinates, Y_coordinates,c=Y_predictions, cmap='Paired')
        plt.title("Hierarchical")
        plt.show()
        r.sleep()



if __name__ == '__main__':
    listener()
    # dbscan()
# WHEN STANDARD SCALE IS REMOVED PLOT IS NOT VISIBLE 
# VICE VERSA IS NOT TRUE
# STANDARD SCALE IS REMOVED BECAUSE AFTER SCALING POINTS ARE NOT CORRECT FOR RVIZMARKER
# inversed = scaler.inverse_transform(scaled)
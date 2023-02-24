#!/usr/bin/env python3

import rospy
import sensor_msgs.msg as sensor_msgs
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import PointCloud2
from pcl_helper import ros_to_pcl ,pcl_to_ros , my_pcl_to_ros , array_to_pointcloud2, dbscan
from visualizer import Visualiser
from matplotlib.animation import FuncAnimation
from sklearn.cluster import DBSCAN, AgglomerativeClustering, KMeans
from sklearn.preprocessing import StandardScaler

class Cluster_creater:
    def __init__(self):
        self.counter = 0
        self.pub = rospy.Publisher("/output/cloud", PointCloud2, queue_size=10)
        self.number_subscriber = rospy.Subscriber("/prius/center_laser/scan/pointcloud2", PointCloud2, self.point_cloud_callback)
        # self.reset_service = rospy.Service("/reset_counter", SetBool, self.callback_reset_counter)
        self.fig, self.ax = plt.subplots()
        self.ln = plt.scatter([], [], 'ro')
        self.x_data, self.y_data = [] , []

    def plot_init(self):
        self.ax.set_xlim(0, 10000)
        self.ax.set_ylim(-7, 7)
        return self.ln

    def plot_graph(self,X):
        ss = StandardScaler()
        X = ss.fit_transform(X)
        min_samples = 5
        eps = .15
        db = DBSCAN(eps=eps, min_samples=min_samples)
        db.fit(X)
        y_pred = db.fit_predict(X)
        x = X[:,0]
        x = np.reshape(x, (8192,1))
        # x =  x.astype(np.float)
        y = X[:,1]
        y = np.reshape(x, (8192,1))
        # y =  y.astype(np.float)
        print(x)
        print(y)
        print(x.shape)
        print(y.shape)
        self.ln.scatter(x, y,c=y_pred, cmap='Paired')
        self.ln.title("DBSCAN")
        self.ln.show()
    
    def point_cloud_callback(self, pcl_msg):
        # self.counter += msg.data
        # new_msg = Int64()
        cloud = ros_to_pcl(pcl_msg)
        cloud_np = np.array(cloud)
        db,X = dbscan(X=cloud_np, eps=.5, min_samples=5)
        self.plot_graph(X)

        return db,X


        # new_msg.data = self.counter
        # self.pub.publish(new_msg)

    # def callback_reset_counter(self, req):
    #     if req.data:
    #         self.counter = 0
    #         return True, "Counter has been successfully reset"
    #     return False, "Counter has not been reset"

if __name__ == '__main__':
    rospy.init_node('number_counter')
    cc= Cluster_creater()
    # db,X = cc.point_cloud_callback()
    # vis = Visualiser()
    # vis.getYaw()
    sub = rospy.Subscriber('/prius/center_laser/scan/pointcloud2', PointCloud2, cc.point_cloud_callback)
    # ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
    # plt.show(block=True) 
    rospy.spin()
import matplotlib.pyplot as plt
import rospy
import tf
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix
import numpy as np

from sensor_msgs.msg import PointCloud2
from matplotlib.animation import FuncAnimation
# from lost_cargo_detection_class import Cluster_creater


class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.scatter([], [], 'ro')
        self.x_data, self.y_data = [] , []

    def plot_init(self):
        self.ax.set_xlim(0, 10000)
        self.ax.set_ylim(-7, 7)
        return self.ln
    
    def getYaw(self, pose):
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2] 
        return yaw  

    def visual_callback(self, msg):
        yaw_angle = self.getYaw(msg.pose.pose)
        self.y_data.append(yaw_angle)
        x_index = len(self.x_data)
        self.x_data.append(x_index+1)
    
    def update_plot(self, frame):
        self.ln.set_data(self.x_data, self.y_data)
        return self.ln


# rospy.init_node('lidar_visual_node')
# vis = Visualiser()

# sub = rospy.Subscriber('/prius/center_laser/scan/pointcloud2', PointCloud2, vis.visual_callback)

# ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
# plt.show(block=True) 
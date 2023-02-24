# import os
# from os import listdir
# from os.path import isfile, join

# path = "/home/cthalia/Downloads/lidar/"

# # pcd_files = [f for f in listdir(path) if isfile(join(path, f))]
# all_pcd = []
# for data in listdir(path):
#   if isfile:
#     all_pcd.append(join(path,data))

# # print(all_pcd)
# for file in all_pcd:
#     cmd = "rosrun pcl_ros pcd_to_pointcloud " + file
#     os.system(cmd)
import pcl
print(pcl.__spec__)
print(pcl.__doc__)
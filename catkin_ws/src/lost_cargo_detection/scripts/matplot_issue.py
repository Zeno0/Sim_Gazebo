import rosbag
import matplotlib.pyplot as plt
import threading

file = rosbag.Bag('/home/cthalia/BAGFILES/SHEKHAR/SHEKHAR/2023-01-31-16-45-17.bag')

ts = []
data = []

def plot():
    plt.plot(ts,data)
    plt.show()

for topic,msg,t in file.read_messages(topics=['/prius/center_laser/scan/pointcloud2']):
    ts.append(t.to_sec())
    data.append(msg.data)

file.close()

thread = threading.Thread(target=plot)
thread.start()



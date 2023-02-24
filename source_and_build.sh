<<<<<<< HEAD
export ROS_DISTRO=noetic
=======
export ROS_DISTRO=$1
>>>>>>> 2ae5b0160e1922fa256e15550422fa2e778f6556
source /opt/ros/$ROS_DISTRO/setup.bash
cd catkin_ws/
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
sudo apt install ros-noetic-ign-ros-control -y
sudo apt install libignition-msgs-dev -y
sudo apt install ros-noetic-effort-controllers -y
sudo apt install ros-noetic-velocity-controllers -y
sudo apt install ros-noetic-velodyne-description -y
sudo apt install ros-noetic-joy -y
sudo apt install ros-noetic-fake-localization -y
sudo ubuntu-drivers autoinstall 
#sudo apt install nvidia-driver-525 -y
sudo apt install nvidia-driver-460 -y # For quadro P2000 , recomended driver
sudo apt install meshlab -y
sudo apt install python3-catkin-tools
#catkin_make
catkin build

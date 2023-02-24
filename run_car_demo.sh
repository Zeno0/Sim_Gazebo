export GAZEBO_MODEL_DATABASE_URI="http://models.gazebosim.org/"
source catkin_ws/devel/setup.bash
roslaunch car_demo demo.launch gpu:=true

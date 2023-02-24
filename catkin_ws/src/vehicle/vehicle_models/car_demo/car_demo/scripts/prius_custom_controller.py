#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from prius_msgs.msg import Control




def move():

    speed = 0.317
    distance = 10

    # Starts a new node
    rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    prius_velocity_publisher = rospy.Publisher('/prius', Control, queue_size=10)
    print(rospy.get_published_topics)
    vel_msg = Twist()
    prius_control_msg = Control()

    #Receiveing the user's input
    print("Let's move your robot")
    #speed = input("Input your speed:")
    #distance = float(input("Type your distance:"))
    #isForward = input("Foward?: ")#True or False

<<<<<<< HEAD:catkin_ws/src/car_demo/car_demo/scripts/prius_custom_controller.py
    speed = .317
    distance = .1
=======
    
>>>>>>> 2ae5b0160e1922fa256e15550422fa2e778f6556:catkin_ws/src/vehicle/vehicle_models/car_demo/car_demo/scripts/prius_custom_controller.py

    while not rospy.is_shutdown():

        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        print(t0)
        current_distance = 0

        #Loop to move the turtle in an specified distance
        while(current_distance < distance):
            # print(current_distance)
            #Publish the velocity
            #Checking if the movement is forward or backwards
            prius_control_msg.throttle = (speed)
            prius_control_msg.steer = -.4
            prius_control_msg.brake = 0
    
            prius_velocity_publisher.publish(prius_control_msg)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            print(t1)
            #Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
        #After the loop, stops the robot
        prius_control_msg.throttle = 0
        prius_control_msg.steer = -.4
        prius_control_msg.brake = 0
        # prius_control_msg.REVERSE = 1
        

        #Force the robot to stop
        prius_velocity_publisher.publish(prius_control_msg)
        break
    # speed = 0

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass


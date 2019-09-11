#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Float64
from math import sin,cos,atan2,sqrt,fabs


#Define a RRBot joint positions publisher for joint controllers.
def i5aubo2_joint_positions_publisher():

    #Initiate node for controlling joint1 and joint2 positions of robot 1 & 2.
    rospy.init_node('joint_positions_node_i5aubo2', anonymous=True)

    #Initiate node for controlling joint1 and joint2 positions of robot 1.

    #Define publishers for each joint position controller commands.
    pub2_j1 = rospy.Publisher('/i5aubo2/shoulder_joint_position_controller/command', Float64, queue_size=10)
    pub2_j2 = rospy.Publisher('/i5aubo2/upperArm_joint_position_controller/command', Float64, queue_size=10)
    pub2_j3 = rospy.Publisher('/i5aubo2/foreArm_joint_position_controller/command', Float64, queue_size=10)
    pub2_j4 = rospy.Publisher('/i5aubo2/wrist1_joint_position_controller/command', Float64, queue_size=10)
    pub2_j5 = rospy.Publisher('/i5aubo2/wrist2_joint_position_controller/command', Float64, queue_size=10)
    pub2_j6 = rospy.Publisher('/i5aubo2/wrist3_joint_position_controller/command', Float64, queue_size=10)
    



    rate = rospy.Rate(50) #100 Hz

    #While loop to have joints follow a certain position, while rospy is not shutdown.
    i = 0
    while not rospy.is_shutdown():

        #Have each joint follow a sine movement of sin(i/100).
        joint1_movement = sin(i/50.)
        joint2_movement = sin(i/60.)
        joint3_movement = cos(i/70.)
        joint4_movement = sin(i/80.)
        joint5_movement = cos(i/100.)
        joint6_movement = sin(i/120.)

        #Publish the same sine movement to each joint.
        pub2_j1.publish(joint1_movement)
        pub2_j2.publish(joint2_movement)
        pub2_j3.publish(joint3_movement)
        pub2_j4.publish(joint4_movement)
        pub2_j5.publish(joint5_movement)
        pub2_j6.publish(joint6_movement)


        i = i+1 #increment i

        rate.sleep() #sleep for rest of rospy.Rate(100)


#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
    try: i5aubo2_joint_positions_publisher()
    except rospy.ROSInterruptException: pass

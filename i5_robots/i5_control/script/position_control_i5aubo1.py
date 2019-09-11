#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Float64
from math import sin,cos,atan2,sqrt,fabs


#Define a RRBot joint positions publisher for joint controllers.
def i5aubo1_joint_positions_publisher():

    #Initiate node for controlling joint1 and joint2 positions of robot 1 & 2.
    rospy.init_node('joint_positions_node_i5aubo1', anonymous=True)

    #Initiate node for controlling joint1 and joint2 positions of robot 1.

    #Define publishers for each joint position controller commands.
    pub1_j1 = rospy.Publisher('/i5aubo1/shoulder_joint_position_controller/command', Float64, queue_size=10)
    pub1_j2 = rospy.Publisher('/i5aubo1/upperArm_joint_position_controller/command', Float64, queue_size=10)
    pub1_j3 = rospy.Publisher('/i5aubo1/foreArm_joint_position_controller/command', Float64, queue_size=10)
    pub1_j4 = rospy.Publisher('/i5aubo1/wrist1_joint_position_controller/command', Float64, queue_size=10)
    pub1_j5 = rospy.Publisher('/i5aubo1/wrist2_joint_position_controller/command', Float64, queue_size=10)
    pub1_j6 = rospy.Publisher('/i5aubo1/wrist3_joint_position_controller/command', Float64, queue_size=10)
    



    rate = rospy.Rate(50) #100 Hz

    #While loop to have joints follow a certain position, while rospy is not shutdown.
    i = 0
    while not rospy.is_shutdown():

        #Have each joint follow a sine movement of sin(i/100).
        joint1_movement = cos(i/50.)
        joint2_movement = sin(i/60.)
        joint3_movement = cos(i/75.)
        joint4_movement = sin(i/90.)
        joint5_movement = cos(i/110.)
        joint6_movement = cos(i/120.)

        #Publish the same sine movement to each joint.
        pub1_j1.publish(joint1_movement)
        pub1_j2.publish(joint2_movement)
        pub1_j3.publish(joint3_movement)
        pub1_j4.publish(joint4_movement)
        pub1_j5.publish(joint5_movement)
        pub1_j6.publish(joint6_movement)


        i = i+1 #increment i

        rate.sleep() #sleep for rest of rospy.Rate(100)


#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
    try: i5aubo1_joint_positions_publisher()
    except rospy.ROSInterruptException: pass

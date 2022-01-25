#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, TransformStamped
from std_msgs.msg import Float32MultiArray
import math
import numpy as np



rospy.init_node("goal_node")
pub = rospy.Publisher("/GOALE", Float32MultiArray, queue_size = 10)
speed= Float32MultiArray()
r = rospy.Rate(10)


while not rospy.is_shutdown():
    #print ("Currently, turtlebot is at (",x,y,")!")
    print('topic_is_live: /GOALE ')
    
    pub.publish(speed)
    r.sleep()    

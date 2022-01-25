#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, TransformStamped
import math

x = 0.0
y = 0.0 
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.transform.translation.x
    y = msg.transform.translation.y

    rot_q = msg.transform.rotation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


rospy.init_node("speed_controller")
sub = rospy.Subscriber("/vicon/TB1/TB1", TransformStamped, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)

speed = Twist()
r = rospy.Rate(10)

#print "Currently, turtlebot is at (",x,y,")!" 
goal = Point()
goal.x = input("Set your x goal:")
goal.y = input("Set your y goal:")

while not rospy.is_shutdown():
    print ("Currently, turtlebot is at (",x,y,")!")
    inc_x = goal.x -x
    inc_y = goal.y -y

    dist_to_goal = math.sqrt(inc_x**2 + inc_y**2)
    angle_to_goal = math.atan2(inc_y, inc_x)
    
    # if(angle_to_goal> math.pi):
    #     angle_to_goal = angle_to_goal - 2*math.pi
    # elif(angle_to_goal<=-math.pi):
    #     angle_to_goal = angle_to_goal + 2*math.pi

    e_th = angle_to_goal - theta
    
    #print(e_th*180/math.pi)
    print(theta*180/math.pi)

    if abs(e_th) >=0.2:
        Kpth = 0.5
        speed.angular.z = Kpth*e_th
        speed.linear.x = 0
        print('CCCCC 1')

    if abs(e_th) <0.2 and abs(dist_to_goal) >=0.1 :
        speed.angular.z = 0
        Kpv = 0.5
        speed.linear.x = Kpv*dist_to_goal
        speed.angular.z = 0
        print('CCCCC 2')
    
    if abs(e_th) <0.19 and abs(e_th) > 0.001: 
        speed.linear.x = 0
        speed.angular.z = 0.2
        print('CCCCC 3')

    if abs(dist_to_goal) <0.1 and abs(e_th) <0.2: 
        speed.linear.x = 0
        speed.angular.z = 0
        print('CCCCC 4')

 
    #print('speed',speed)
    pub.publish(speed)
    r.sleep()    

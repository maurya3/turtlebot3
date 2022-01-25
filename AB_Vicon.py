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
sub = rospy.Subscriber("/odom", TransformStamped, newOdom)
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
    
    alp = -theta + angle_to_goal
    bta = -theta - alp
    
    print(theta*180/math.pi)
    if theta <= 180:
        theta = theta +180
    
    if(angle_to_goal> math.pi):
        angle_to_goal = angle_to_goal - math.pi
    elif(angle_to_goal<=-math.pi):
        angle_to_goal = angle_to_goal + math.pi

    if(theta > math.pi):
        theta = theta - math.pi
    elif(theta<=-math.pi):
        theta = theta + 2*math.pi


    Kpv =0.2
    Kp_bta = -0.3
    kp_al = 0.3

    speed.angular.z = kp_al*alp + Kp_bta* bta
    speed.linear.x = Kpv*dist_to_goal
    
    print('speed',speed)
    pub.publish(speed)
    r.sleep()    

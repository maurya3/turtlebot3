#! /usr/bin/env python3

from operator import mod
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, TransformStamped
from std_msgs.msg import Float32MultiArray
import math
import numpy as np

x = 0.0
y = 0.0 
theta = 0.0

speed1 = Twist()

X = np.array([0,1,0])


INT_V =0
INT_w = 0


def newOdom(msg):
    global x
    global y
    global theta

    x = msg.transform.translation.x
    y = msg.transform.translation.y

    rot_q = msg.transform.rotation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def Goal_Callback(msg):
    global GoPt
    GoPt = msg.data
    rospy.loginfo("Goal_Received")

rospy.init_node("speed_controller")
sub = rospy.Subscriber("/vicon/TB1/TB1", TransformStamped, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
SubGoal = rospy.Subscriber("/GOAL",Float32MultiArray,Goal_Callback)


speed = Twist()
r = rospy.Rate(20)

 
goal = Point()
goal.x = input("Set your x goal:")
goal.y = input("Set your y goal:")

while not rospy.is_shutdown():
    
    PKv = 10
    PKw = 10

    IKv = 1.0 
    IKw = 2.0

    DKv = 0.1
    DKw = 0.8

    e_p = 0
    d_p = 0
    Vc_max = 0.17
    Wc_max = 0.6    
    
    INT_MAX = 0
    INT_MIN = 0
    
    print ("Currently, turtlebot is at (",x,y,")!")
    x0 = goal.x 
    y0 = goal.y
    #Vc, Wc = go_to_A2B(x,y,theta,goal1x,goal1y) # meter, meter, radian(-pi,pi],meter, meter
    
    beta = math.atan2(y0-y,x0-x) # (-pi,pi])
    
    alpha = beta - theta # (-2pi,2pi)
    d = math.sqrt((x0 -x)**2 + (y0 -y)**2)


    if(alpha<=-math.pi):
        alpha = alpha + 2.0*math.pi
    elif(alpha>math.pi):
        alpha = alpha - 2.0*math.pi

    INT_V = INT_V  + d
    INT_w = INT_w + alpha

    if INT_V > INT_MAX:
        INT_V = INT_MAX
    if INT_V < INT_MAX:
        INT_V = INT_MIN

    if INT_w > INT_MAX:
        INT_w = INT_MAX
    if INT_w < INT_MAX:
        INT_w = INT_MIN



    Wc = PKw * alpha  + IKw * INT_w  + DKv* (alpha - e_p)
    Vc = PKv * d  + IKv * INT_V + DKv * (d - d_p)
    
    e_p = alpha
    d_p = d
    INT_V = d
    INT_w = alpha

    if(Vc<0):
        Vc = 0
    print(Wc)
    # if tracking multiple points tgen comment it
    if(d<0.05):
        Vc = 0
        Wc = 0

    if(Vc>Vc_max):
        Vc = Vc_max
    elif(Vc<-Vc_max):
        Vc = -Vc_max
    
    if(Wc>Wc_max):
        Wc = Wc_max
    elif(Wc<-Wc_max):
        Wc = -Wc_max
    
    print(Vc,Wc)
    #print(x,y)

    K1 = 1
    K2 = 1
    speed1.linear.x  = Vc * K1
    speed1.angular.z = Wc * K2
    
    pub.publish(speed1)
    r.sleep() 

    # except:
    #     pass   

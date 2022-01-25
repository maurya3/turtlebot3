#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, TransformStamped
import math
import numpy as np
import time

x = 0.0
y = 0.0 
theta = 0.0

X = np.array([0,1,0])

def go_to_A2B(x,y,th,x0,y0):
    #print ("Currently, turtlebot is at ",(round(x,2),round(y,2),round(th*180.0/math.pi)))
    Vc = 0 #[0.26,0.26]
    Wc = 0 #[-1.82,1.82] 104.27 deg/sec

    Vc_max = 0.12
    Wc_max = 0.5    

    Kv = 0.09 #0.25
    Kw = 1.2 #1.16

    beta = math.atan2(y0-y,x0-x) # (-pi,pi])
    alpha = beta - th # (-2pi,2pi)

    if(alpha<=-math.pi):
        alpha = alpha + 2.0*math.pi
    elif(alpha>math.pi):
        alpha = alpha - 2.0*math.pi

    Wc = Kw*alpha
    
    if(Wc>Wc_max):
        Wc = Wc_max
    elif(Wc<-Wc_max):
        Wc = -Wc_max
    #print(Wc)
    
    d = math.sqrt((x0 -x)**2 + (y0 -y)**2)
    # logic 1
    #Vc = Kv*d

    # logic 2
    #Vc = abs(Kv*d*math.cos(alpha))

    # logic 2
    Vc = Kv*d*math.cos(alpha)
    if(Vc<0):
        Vc = 0

    # if tracking multiple points tgen comment it
    '''if(d<0.05):
        Vc = 0
        Wc = 0'''

    if(Vc>Vc_max):
        Vc = Vc_max
    elif(Vc<-Vc_max):
        Vc = -Vc_max

    '''Vc = 0
    Wc = 0'''

    return Vc, Wc

def traj_circle(cx,cy,th0,t):
    cx = 0
    cy = 0
    r = 1.0
    vel = 0.075
    omega = vel/r
    x0 = cx + r*math.cos(th0 + 0.5*math.pi + omega*t)
    y0 = cy + r*math.sin(th0 + 0.5*math.pi + omega*t)

    return x0, y0

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

cx = x
cy = y
th0 = theta
time_now = time.time()

while not rospy.is_shutdown():
    #print ("Currently, turtlebot is at (",x,y,")!")
    t = time.time() - time_now
    goal.x, goal.y = traj_circle(cx,cy,th0,t)

    Vc, Wc = go_to_A2B(x,y,theta,goal.x,goal.y) # meter, meter, radian(-pi,pi],meter, meter
    print(Vc,Wc)
    #print(x,y)

    speed.linear.x  = Vc
    speed.angular.z = Wc
    
    pub.publish(speed)
    r.sleep()    

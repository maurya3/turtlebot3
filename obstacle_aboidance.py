#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, TransformStamped
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
import math
import numpy as np

x = 0.0
y = 0.0 
theta = 0.0
speed1 = Twist()


def laser_cb(data):
    global scan
    scan = data.ranges
    

def repu_force(xc,yc,xo,yo,ro_thres,eta=0):
    dx = xo -xc
    dy = yo - yc
    d= math.sqrt((xo - xc)**2  + (yo - yc)**2)

    if(d<ro_thres):
        grad_qx = (xc - xo)/d
        grad_qy = (yc - yo)/d
        Fx_rep = eta *((1/d) - (1/ro_thres))*(1/d**2)*grad_qx
        Fy_rep = eta *((1/d) - (1/ro_thres))*(1/d**2)*grad_qy
    else:
        Fx_rep = 0
        Fy_rep = 0

    return Fx_rep, Fy_rep



def attr_force(xc,yc,xg,yg,d_thres,zeta=1):
    dx = xg-xc
    dy = yg-yc

    d= math.sqrt((xg - xc)**2  + (yg - yc)**2)
    if d <= d_thres:
        Fx_attr =  -zeta*(xc - xg)
        Fy_attr = -zeta*(yc - yg)


    if d > d_thres:
        Fx_attr = -d_thres*(1/d)*zeta*(xc - xg)
        Fy_attr = -d_thres*(1/d)*zeta*(yc - yg)

   
    return Fx_attr, Fy_attr




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
subL = rospy.Subscriber('/scan',LaserScan,laser_cb)


speed = Twist()
r = rospy.Rate(10)

#print "Currently, turtlebot is at (",x,y,")!" 
goal = Point()
goal.x = input("Set your x goal:")
goal.y = input("Set your y goal:")


def go_to_A2B(x,y,th,x0,y0):
    
    #print ("Currently, turtlebot is at ",(round(x,2),round(y,2),round(th*180.0/math.pi)))
    Vc = 0 #[0.26,0.26]
    Wc = 0 #[-1.82,1.82] 104.27 deg/sec

    Vc_max = 0.17
    Wc_max = 0.19    

    Kv = 0.15 #0.25
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
    Vc = Kv*d*math.cos(alpha)*0.8
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


def obs_theta(xc,yc,xo,yo):
    th_o = math.atan2(yo-yc,xo-xc)

    return th_o


while not rospy.is_shutdown():
        #print ("Currently, turtlebot is at (",x,y,")!")
        #goal1x = GoPt[0]
        #goal1y = GoPt[1]

        xc = x
        yc = y
        xg = goal.x
        yg = goal.y
        xo = 0.3
        yo = 0.2

        goal_reached = False
        dist2goal = math.sqrt((xg -x)**2 + (yg -y)**2)

        Vc = 0
        Wc = 0

        if(dist2goal<0.08):
            Vc = 0
            Wc = 0

        else:
            ro_thres = 0.3
            d_thres = 1.0
             

        Fx_rep, Fy_rep = repu_force(xc,yc,xo,yo,ro_thres,eta=0.001) 
        Fx_attr, Fy_attr  = attr_force(xc,yc,xg,yg,d_thres,zeta=0.65)
        

        beta = math.atan2(yc-yg,xc-xg) # (-pi,pi])
        alpha1 = beta - theta # (-2pi,2pi)

        # if(alpha1<=-math.pi):
        #     alpha1 = alpha1 + 2.0*math.pi
        # elif(alpha1>math.pi):
        #     alpha1 = alpha1 - 2.0*math.pi


        th_o = obs_theta(xc,yc,xo,yo)

        TFx = Fx_rep*math.cos(th_o) + Fx_attr*math.cos(alpha1)

        TFy = Fy_rep*math.sin(th_o) + Fy_attr*math.sin(alpha1)

        
        if TFx>0:
            delta = math.atan2(TFy,TFx)

        elif TFx<=0:
            delta =math.pi +  math.atan2(TFy,TFx)


        nextx = xc + 0.1* math.cos(delta)
        nexty = yc + 0.1*math.sin(delta)

        print(x,y,nextx,nexty)

       
        #nextx = 0.0
        #nexty = 0.0

        Vc, Wc = go_to_A2B(x,y,theta,nextx,nextx) # meter, meter, radian(-pi,pi],meter, meter

        if math.sqrt((xg - nextx)**2 + (yg - nextx)**2) <=0.2:
            Vc = 0
            Wc = 0
        
        print(Vc,Wc)
        #print(x,y)

        K12 = 3.0
        speed1.linear.x  = Vc * K12
        speed1.angular.z = Wc * K12
        print(speed1)
    
        pub.publish(speed1)
        r.sleep() 

    # except:
    #     pass   

#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, TransformStamped
from std_msgs.msg import Float32MultiArray
import math
import numpy as np
import csv
import time

x = 0.0
y = 0.0 
theta = 0.0
time_p = time.time()

with open('3G_2109_ANM_02.csv', 'a') as csvfile:
    fieldnames = ['x1', 'y1', 'x2','y2', 'x3', 'y3', 'x4','y4', 'x5', 'y5','gx1','gy1','gx2','gy2','gx3','gy3','gx4','gy4','gx5','gy5','time']
    my_writer = csv.DictWriter(csvfile, delimiter = ' ',fieldnames= fieldnames)
    my_writer.writeheader()
    

X = np.array([0,1,0])
def detect_purtb(xc,yc,gx,gy):
    d = math.sqrt((gx -xc)**2 + (gy -yc)**2)
    if d > 0.5:
        gx = xc
        gy = yc

    else:
        gx = gx
        gy = gy

    return gx,gy

def go_to_A2B(x,y,th,x0,y0):
    #print ("Currently, turtlebot is at ",(round(x,2),round(y,2),round(th*180.0/math.pi)))
    Vc = 0 #[0.26,0.26]
    Wc = 0 #[-1.82,1.82] 104.27 deg/sec

    Vc_max = 0.12
    Wc_max = 0.3    

    Kv = 0.12 #0.25
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
        Vc = Vc*0.5

    # if tracking multiple points tgen comment it
    if(d<0.06):
        Vc = 0
        Wc = 0

    if(Vc>Vc_max):
        Vc = Vc_max
    elif(Vc<-Vc_max):
        Vc = -Vc_max

    '''Vc = 0
    Wc = 0'''

    return Vc, Wc

def newOdom1(msg):
    global x1
    global y1
    global theta1

    x1 = msg.transform.translation.x
    y1 = msg.transform.translation.y

    rot_q = msg.transform.rotation
    (roll, pitch, theta1) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def newOdom2(msg):
    global x2
    global y2
    global theta2

    x2 = msg.transform.translation.x
    y2 = msg.transform.translation.y

    rot_q = msg.transform.rotation
    (roll, pitch, theta2) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def newOdom3(msg):
    global x3
    global y3
    global theta3

    x3 = msg.transform.translation.x
    y3 = msg.transform.translation.y

    rot_q = msg.transform.rotation
    (roll, pitch, theta3) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def newOdom4(msg):
    global x4
    global y4
    global theta4

    x4 = msg.transform.translation.x
    y4 = msg.transform.translation.y

    rot_q = msg.transform.rotation
    (roll, pitch, theta4) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def newOdom5(msg):
    global x5
    global y5
    global theta5
 

    x5 = msg.transform.translation.x
    y5 = msg.transform.translation.y

    rot_q = msg.transform.rotation
    (roll, pitch, theta5) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def Goal_Callback(msg):
    global GoPt
    GoPt = msg.data
    rospy.loginfo("Goal_Received")


rospy.init_node("speed_controller")

sub1 = rospy.Subscriber("/vicon/TB1/TB1", TransformStamped, newOdom1)
sub2 = rospy.Subscriber("/vicon/TB2/TB2", TransformStamped, newOdom2)
sub3 = rospy.Subscriber("/vicon/TB3/TB3", TransformStamped, newOdom3)
sub4 = rospy.Subscriber("/vicon/TB4/TB4", TransformStamped, newOdom4)
sub5 = rospy.Subscriber("/vicon/TB5/TB5", TransformStamped, newOdom5)
SubGoal = rospy.Subscriber("/GOAL",Float32MultiArray,Goal_Callback)

pub1 = rospy.Publisher("/TB1/cmd_vel", Twist, queue_size = 3)
pub2 = rospy.Publisher("/TB2/cmd_vel", Twist, queue_size = 3)
pub3 = rospy.Publisher("/TB3/cmd_vel", Twist, queue_size = 3)
pub4 = rospy.Publisher("/TB4/cmd_vel", Twist, queue_size = 3)
pub5 = rospy.Publisher("/TB5/cmd_vel", Twist, queue_size = 3)


speed1 = Twist()
speed2 = Twist()
speed3 = Twist()
speed4 = Twist()
speed5 = Twist()

r = rospy.Rate(10)

#print "Currently, turtlebot is at (",x,y,")!" 
# goal1 = Point()
# goal1.x = input("Set your x1 goal:")
# goal1.y = input("Set your y1 goal:")

# goal2 = Point()
# goal2.x = input("Set your x2 goal:")
# goal2.y = input("Set your y2 goal:")

# goal3 = Point()
# goal3.x = input("Set your x3 goal:")
# goal3.y = input("Set your y3 goal:")

# goal4 = Point()
# goal4.x = input("Set your x4 goal:")
# goal4.y = input("Set your y4 goal:")

# goal5 = Point()
# goal5.x = input("Set your x4 goal:")
# goal5.y = input("Set your y4 goal:")


while not rospy.is_shutdown():
    try:
        #print ("Currently, turtlebot is at (",x,y,")!")
        goal1x = GoPt[0] #----1
        goal1y = GoPt[1]
        goal2x = GoPt[2] #----2
        goal2y = GoPt[3]
        goal3x = GoPt[4] #----3
        goal3y = GoPt[5]
        goal4x = GoPt[6] #----4
        goal4y = GoPt[7]
        goal5x = GoPt[8] #----5
        goal5y = GoPt[9]
        a_count= GoPt[10]

        if a_count ==2:
            goal2x,goal2y = detect_purtb(x2,y2,goal2x,goal2y)
            goal4x,goal4y = detect_purtb(x4,y4,goal4x,goal4y)

        Vc1, Wc1 = go_to_A2B(x1,y1,theta1,goal1x,goal1y) # meter, meter, radian(-pi,pi],meter, meter
        Vc2, Wc2 = go_to_A2B(x2,y2,theta2,goal2x,goal2y)
        Vc3, Wc3 = go_to_A2B(x3,y3,theta3,goal3x,goal3y)
        Vc4, Wc4 = go_to_A2B(x4,y4,theta4,goal4x,goal4y)
        Vc5, Wc5 = go_to_A2B(x5,y5,theta5,goal5x,goal5y)
        
        print(Vc1,Wc1,Vc2,Wc2,Vc3,Wc3,Vc4,Wc4,Vc5,Wc5)
        
        k=1.3
        speed1.linear.x  = Vc1*k
        speed1.angular.z = Wc1*k

        speed2.linear.x  = Vc2*k
        speed2.angular.z = Wc2*k

        speed3.linear.x  = Vc3*k
        speed3.angular.z = Wc3*k

        speed4.linear.x  = Vc4*k
        speed4.angular.z = Wc4*k

        speed5.linear.x  = Vc5*k
        speed5.angular.z = Wc5*k
        
        pub1.publish(speed1)
        pub2.publish(speed2)
        pub3.publish(speed3)
        pub4.publish(speed4)
        pub5.publish(speed5)
        time_c = time.time()
        dtime = time_c -time_p

        with open('3G_2109_ANM_02.csv','a') as csv_file:
            pass

	    with open('3G_2109_ANM_02.csv', 'a') as csvfile:
             fieldnames = ['x1', 'y1', 'x2','y2', 'x3', 'y3', 'x4','y4', 'x5', 'y5','gx1','gy1','gx2','gy2','gx3','gy3','gx4','gy4','gx5','gy5','time']
             my_writer = csv.DictWriter(csvfile, delimiter = ' ',fieldnames= fieldnames)
             my_writer.writerow({'x1':x1, 'y1':y1, 'x2':x2,'y2':y2, 'x3':x3, 'y3':y3, 'x4':x4,'y4':y4, 'x5':x5, 'y5':y5,'gx1':goal1x,'gy1':goal1y,'gx2':goal2x,'gy2':goal2y,'gx3':goal3x,'gy3':goal3y,'gx4':goal4x,'gy4':goal4y,'gx5':goal5x,'gy5':goal5y,'time':dtime})

        r.sleep()   
        
    except:
        pass 

#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, TransformStamped
import math
import numpy as np

import array as arr
import math
import matplotlib.pyplot as plt
import numpy as np
x= 0.9633649462029878
y= 0.4073163540455491
theta = 4 * 3.14/180

start = 0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.transform.translation.x
    y = msg.transform.translation.y

    rot_q = msg.transform.rotation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


rospy.init_node("speed_controller")
 
speed = Twist()
r = rospy.Rate(10)




def a2b_ddr_traj_1(Pa,Pb,Vc_max,Wc_max):

    # initial coordinates
    x1  = Pa[0]
    y1  = Pa[1]
    th1 = Pa[2]

    # final coordinates
    x2  = Pb[0]
    y2  = Pb[1]
    th2 = Pb[2]

    # direction of point B from A & distance D of B from A
    delX = x2 - x1
    delY = y2 - y1
    thi = math.atan2(delY,delX)
    D = math.sqrt(delX**2 + delY**2)

    # time distribution for maneuver
    # T1: turn towards goal
    # T2: turn towards goal
    # T3: turn towards goal
    # T : total time to reach B from A
    T1 = abs((3*(thi-th1))/(2*Wc_max))
    T2 = (3*D)/(2*Vc_max)
    T3 = abs((3*(th2-thi))/(2*Wc_max))
    t1 = T1
    t2 = T1 + T2
    t3 = T1 + T2 + T3
    T  = t3
    
    dt = 0.1
    t = arr.array('d',[0])
    while (t[len(t)-1]<T):
        t.append(t[len(t)-1]+dt)

    # reference trajectory (x0, y0, th0, v0, w0)
    x0  = arr.array('d',[x1])
    y0  = arr.array('d',[y1])
    th0 = arr.array('d',[th1])
    v0  = arr.array('d',[0])
    w0  = arr.array('d',[0])
    
    for i in range(0,len(t)-1):
        if(t[i]<=t1): # plan for turn towards goal B while staying at A
            a0 = th1
            a1 = 0
            a2 = (3*(thi-th1))/(t1**2)
            a3 = (2*(th1-thi))/(t1**3)
            tht = a0 + a1*t[i] + a2*t[i]**2 + a3*t[i]**3
            xt  = x1
            yt  = y1
            vt  = 0
            wt  = a1 + 2*a2*t[i] + 3*a3*t[i]**2
        elif((t[i]>t1) and (t[i]<=t2)): # plan to go B from A
            b0 = 0
            b1 = 0
            b2 = 3*D/(t2-t1)**2
            b3 = -2*D/(t2-t1)**3
            dt = b0 + b1*(t[i]-t1) + b2*(t[i]-t1)**2 + b3*(t[i]-t1)**3
            xt  = x1 + dt*math.cos(thi)
            yt  = y1 + dt*math.sin(thi)
            tht = thi
            vt  = b1 + 2*b2*(t[i]-t1) + 3*b3*(t[i]-t1)**2
            wt  = 0
        elif((t[i]>t2) and (t[i]<=t3)): # plan for turn towards goal th2 while staying at B
            c0 = thi
            c1 = 0
            c2 = (3*(th2-thi))/((t3-t2)**2)
            c3 = (2*(thi-th2))/((t3-t2)**3)
            tht = c0 + c1*(t[i]-t2) + c2*(t[i]-t2)**2 + c3*(t[i]-t2)**3
            xt  = x2
            yt  = y2
            vt  = 0
            wt  = c1 + 2*c2*(t[i]-t2) + 3*c3*(t[i]-t2)**2
        else: # if trajectory planning is complete
            xt  = x2
            yt  = y2
            tht = th2
            vt  = 0
            wt  = 0
        
        x0.append(xt)
        y0.append(yt)
        th0.append(tht)
        v0.append(vt)
        w0.append(wt)

    return t,x0,y0,th0,v0,w0

def traj_tracker_1(xr,yr,thr,vr,wr,xc,yc,thc):
    pc = np.array([xc, yc, thc])
    pr = np.array([xr, yr, thr])
    Te = np.array([[math.cos(thc),math.sin(thc),0], [-math.sin(thc),math.cos(thc),0], [0, 0, 1]])
    pe = np.dot(Te,(pr-pc))

    xe  = pe[0]
    ye  = pe[1]
    the = pe[2]

    # tune the values of Kx and Ky
    Kx  = 1
    Ky  = 1
    Kth = 2*1*math.sqrt(Ky)

    v = vr*math.cos(the) + Kx*xe
    w = wr + vr*(Ky*ye + Kth*math.sin(the))
    return v, w

def deg2rad(deg):
    return deg*math.pi/180.0

def rad2deg(rad):
    return rad*180.0/math.pi

# initial coordinate at point A
x1 = x
y1 = y
th1 = theta

# final coordinate at point B
#print "Currently, turtlebot is at (",x,y,")!"
goal = Point()
goal.x = input("Set your x goal (m): ")
goal.y = input("Set your y goal (m): ")
th2    = input("Set your th goal (deg): ")

traj_i = 0
x2 = goal.x
y2 = goal.y
th2 = deg2rad(th2)

Pa = arr.array('d',[x1,y1,th1])
Pb = arr.array('d',[x2,y2,th2])
Vc_max = 0.12 # maximum linear velocity of robot
Wc_max = deg2rad(30) # maximum angular velocity of robot

# calculating desired trajectory
t,x0,y0,th0,v0,w0 = a2b_ddr_traj_1(Pa,Pb,Vc_max,Wc_max)

# displaying desired trajectory
plt.subplot(411)
plt.plot(t,x0,label='x0',linestyle="--")
plt.plot(t,y0,label='y0',linestyle="--")
plt.legend()
plt.xlabel('t (sec)')
plt.ylabel('x0 (m), y0 (m)')
plt.grid(color = 'green', linestyle = '--', linewidth = 0.5)

plt.subplot(412)
plt.plot(t,th0,label='th0',linestyle="--")
plt.legend()
plt.xlabel('t (sec)')
plt.ylabel('th0 (rad)')
plt.grid(color = 'green', linestyle = '--', linewidth = 0.5)

plt.subplot(413)
plt.plot(t,v0,label='v0',linestyle="--")
plt.plot(t,w0,label='w0',linestyle="--")
plt.legend()
plt.xlabel('t (sec)')
plt.ylabel('v0 (m/sec), w0 (rad/sec)')
plt.grid(color = 'green', linestyle = '--', linewidth = 0.5)

plt.subplot(414)
plt.plot(x0,y0,label='path',linestyle="--")
plt.legend()
plt.xlabel('x0 (m)')
plt.ylabel('y0 (m)')
plt.grid(color = 'green', linestyle = '--', linewidth = 0.5)

#plt.show()

x = 0.0
y = 0.0 
theta = 0.0

X = np.array([0,1,0])

def go_to_A2B(x,y,th,x0,y0):
    #print ("Currently, turtlebot is at ",(round(x,2),round(y,2),round(th*180.0/math.pi)))
    Vc = 0 #[0.26,0.26]
    Wc = 0 #[-1.82,1.82] 104.27 deg/sec

    Vc_max = 0.17
    Wc_max = 0.9    

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
    Vc = Kv*d*math.cos(alpha)
    if(Vc<0):
        Vc = 0

    # if tracking multiple points tgen comment it
    if(d<0.05):
        Vc = 0
        Wc = 0

    if(Vc>Vc_max):
        Vc = Vc_max
    elif(Vc<-Vc_max):
        Vc = -Vc_max

    '''Vc = 0
    Wc = 0'''

    return Vc, Wc
    
 

while not rospy.is_shutdown():
     
    '''try:'''
    sub = rospy.Subscriber("/vicon/TB_2/TB_2", TransformStamped, newOdom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
    print (x,y,theta)
    
    #Vc, Wc = go_to_A2B(x,y,theta,goal.x,goal.y) # meter, meter, radian(-pi,pi],meter, meter
    xr  = x0[traj_i]
    yr  = y0[traj_i]
    thr = th0[traj_i]
    vr  = v0[traj_i]
    wr  = w0[traj_i]
    traj_i = traj_i + 1
    if(traj_i>len(x0)-1):
        traj_i = len(x0)-1

    x0 = 1
    y0 = 0

    Vc,Wc = traj_tracker_1(xr,yr,thr,vr,wr,x,y,theta)
    #Vc, Wc = go_to_A2B(x,y,rad2deg(theta),x0,y0)
    #print(Vc,Wc)
    #print(x,y)

    speed.linear.x  = Vc
    speed.angular.z = Wc
    
    pub.publish(speed)
    r.sleep()

    '''except:
        print('wait wait !!! ')'''
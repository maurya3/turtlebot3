import array as arr
import math
import matplotlib.pyplot as plt
import numpy as np

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
    Kx  = 10
    Ky  = 0.64
    Kth = 1.6 #2*zeta*math.sqrt(Ky)

    v = vr*math.cos(the) + Kx*xe
    w = wr + vr*(Ky*ye + Kth*math.sin(the))
    return v, w

def deg2rad(deg):
    return deg*math.pi/180.0

def rad2deg(rad):
    return rad*180.0/math.pi


# initial coordinate at point A
x1 = 0
y1 = 0
th1 = deg2rad(0)

# final coordinate at point B
x2 = 2
y2 = 3
th2 = deg2rad(0)

Pa = arr.array('d',[x1,y1,th1])
Pb = arr.array('d',[x2,y2,th2])
Vc_max = 0.25 # maximum linear velocity of robot
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

plt.show()

v,w = traj_tracker_1(1,2,0.5,0.15,0.3,1.02,1.92,0.485)

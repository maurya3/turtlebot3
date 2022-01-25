"""
Move to specified pose
"""

import matplotlib.pyplot as plt
import numpy as np
from random import random
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, TransformStamped
import math

# simulation parameters
Kp_rho = 0.9
Kp_alpha = 0.15
Kp_beta = -0.03
dt = 0.01

show_animation = True

def newOdom(msg):
    global xc
    global yc
    global thetac

    xc = msg.transform.translation.x
    yc = msg.transform.translation.y

    rot_q = msg.transform.rotation
    (roll, pitch, thetac) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


 
def move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal):
    v = 0
    w = 0
    """
    rho is the distance between the robot and the goal position
    alpha is the angle to the goal relative to the heading of the robot
    beta is the angle between the robot's position and the goal position plus the goal angle

    Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards the goal
    Kp_beta*beta rotates the line so that it is parallel to the goal angle
    """
    xc = x_start
    yc = y_start
    theta = theta_start

    x_diff = x_goal - xc
    y_diff = y_goal - yc

    x_traj, y_traj = [], []

    rho = np.hypot(x_diff, y_diff)
    if rho > 0.001:
        x_traj.append(xc)
        y_traj.append(yc)

        x_diff = x_goal - xc
        y_diff = y_goal - yc

        # Restrict alpha and beta (angle differences) to the range
        # [-pi, pi] to prevent unstable behavior e.g. difference going
        # from 0 rad to 2*pi rad with slight turn

        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff)
                    - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi

        v = Kp_rho * rho
        w = Kp_alpha * alpha + Kp_beta * beta

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v

        theta = theta + w * dt
        xc = xc + v * np.cos(theta) * dt
        yc = yc + v * np.sin(theta) * dt

        print(v,w)

    return v, w


rospy.init_node("speed_controller")
sub = rospy.Subscriber("/vicon/TB1/TB1", TransformStamped, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)

speed = Twist()
r = rospy.Rate(10)

#print "Currently, turtlebot is at (",x,y,")!" 
goal = Point()
goal.x = input("Set your x goal:")
goal.y = input("Set your y goal:")

x = goal.x
y = goal.y

        

while not rospy.is_shutdown():

    #print ("Currently, turtlebot is at (",x,y,")!")

    #Vc, Wc = go_to_A2B(x,y,theta,goal.x,goal.y) # meter, meter, radian(-pi,pi],meter, meter
    #print(Vc,Wc)
    #print(x,y)

    #speed.linear.x  = Vc
    #speed.angular.z = Wc
    

    x_start = xc
    y_start = yc
    theta_start = thetac
    x_goal = goal.x
    y_goal = goal.y
    theta_goal = 0.2 * np.pi
    print("Initial x: %.2f m\nInitial y: %.2f m\nInitial theta: %.2f rad\n" %
        (x_start, y_start, theta_start))
    print("Goal x: %.2f m\nGoal y: %.2f m\nGoal theta: %.2f rad\n" %
        (x_goal, y_goal, theta_goal))
    
    v,w = move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal)

    speed.linear.x  = v
    speed.angular.z = w
        
    pub.publish(speed)
    r.sleep() 
   





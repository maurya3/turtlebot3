from numpy.core.fromnumeric import size
import rospy
from sensor_msgs.msg import LaserScan
import math
import matplotlib.pyplot as plt
import cv2
import matplotlib
matplotlib.use('TkAgg')
import time
import numpy as np

scan= []


def laser_cb(data):
    global scan
    scan = data.ranges
    #rospy.loginfo('.....')

rospy.init_node('\data_laser',anonymous=True)
sub = rospy.Subscriber('/scan',LaserScan,laser_cb)
rospy.Rate(15)

def _scan(scan):
    X = []
    Y = []
    for i in range (size(scan)):
        x= scan[i] * math.cos(i * math.pi / 180.0)
        y = scan[i] * math.sin(i * math.pi / 180.0)
        X.append(x)
        Y.append(y)

    return X,Y

def filter_ranges(X, Y):
    x_f = []
    y_f = []

    for i in range(size(X)):
        d = math.sqrt(X[i]**2 + Y[i]**2)
        if((d>0.05) and (d<=4.0)):
            x_f.append(X[i])
            y_f.append(Y[i])
    
    return x_f, y_f


def detect_obs(scan):
    r_obs = []
    th_obs = []
    X_obs = []
    Y_obs = []

    d_min = 0.20
    d_max = 1.0
    
    for i in range (size(scan)):
        d_obs = scan[i]
        if((d_obs>d_min) and (d_obs<=d_max)):
            r = d_obs
            th = (math.pi/360.0)*i
            x= scan[i] * math.cos(i * math.pi / 180.0)
            y = scan[i] * math.sin(i * math.pi / 180.0)
            r_obs.append(r)
            th_obs.append(th)
            X_obs.append(x)
            Y_obs.append(y)
    return r_obs, th_obs, X_obs, Y_obs

 

def get_patch_n_obs(scan):
    index_obj = 0
    obj_arr = np.zeros_like(scan) # 0 for no object, 1, 2, 3, i for i_th object

    d_min = 0.05 # min distance to detect object
    d_max = 1.0 # maximum distance upto which object is to be detected
    
    d_obs = scan[0]
    obs_present = (d_obs>d_min) and (d_obs<=d_max)
    if(obs_present):
        index_obj = index_obj + 1
        obj_arr[0] = index_obj
    else:
        obj_arr[0] = 0
    
    for i in range (1,np.size(scan)):
        d_obs = scan[i]
        obs_present = (d_obs>=d_min) and (d_obs<=d_max)

        if(obs_present):
            #obj_arr[i] = 1
            if(obj_arr[i-1]==0):
                index_obj = index_obj + 1
            obj_arr[i] = index_obj

        else:
            obj_arr[i] = 0
        
    # check first and last index for object continuity
    singularity = (obj_arr[0]>0) and (obj_arr[np.size(scan)-1]>0)
    
    if(singularity):
        singularity_i = np.size(scan)-1
        while(obj_arr[singularity_i]>0):
            obj_arr[singularity_i] = 1
            singularity_i = singularity_i - 1

    n_obj = np.max(obj_arr)

    return n_obj, obj_arr


def detect_obs_edges(scan):
    n_obj = 0
    index_obj = 0
    obj_arr = np.zeros(360)
    obj_edge = np.zeros(360)

    r_obs = [] # distance of objects
    th_obs = [] # angle of object
    X_obs = [] # x coordinate of object
    Y_obs = [] # y coordinate of object

    d_min = 0.20 # min distance to detect object
    d_max = 1.0 # maximum distance upto which object is to be detected

    # detect object
    for i in range (size(scan)):
        d_obs = scan[i]
        obs_present = (d_obs>d_min) and (d_obs<=d_max)
        if(obs_present):
            obj_arr[i] = 1
        else:
            obj_arr[i] = 0
    
    # detect object edges
    # append zero at beginning and at end
    obj_arr_temp = obj_arr
    obj_arr_temp = np.concatenate(([0],obj_arr_temp,[0]), axis=0)

    for i in range (size(obj_arr_temp)-1):
        diff = obj_arr_temp[i] - obj_arr_temp[i-1]
        if(diff==-1):
            obj_edge[i] = 1
        elif(diff==1):
            obj_edge[i+1] = 1
        else:
            obj_edge[i] = 0
    return obj_arr


fig = plt.figure(figsize=(5,5))
fig.canvas.draw()

def main():
    while not rospy.is_shutdown():
        timec = time.time()
        X = 0
        Y = 0
        X,Y = _scan(scan)

        r_obs, th_obs, X_obs, Y_obs = detect_obs(scan)
        fig.clf()
        plt.plot([0,3],[0, 0],"g-") 

        
        # figure 1
        # # plot laser scan
        plt.plot([X, np.zeros(np.size(X))],[Y, np.zeros(np.size(Y))],"b--")

        # plot obstacle
        plt.plot([X_obs, np.zeros(np.size(X_obs))],[Y_obs, np.zeros(np.size(Y_obs))],"r--") 

        # figure 2
        # plot laser scan
        #plt.plot(X, Y, "b--")
        x_f, y_f = filter_ranges(X, Y)
        plt.scatter(x_f, y_f)

        # plot obstacle
        plt.plot([X_obs, np.zeros(np.size(X_obs))],[Y_obs, np.zeros(np.size(Y_obs))],"r--")
        try:
            n_obj, obj_patch = get_patch_n_obs(scan)
            print(n_obj, obj_patch)
        except:
            pass

        xy_lim = 4
        plt.xlim(-xy_lim,xy_lim)
        plt.ylim(-xy_lim,xy_lim)
        plt.grid(True)
        plt.pause(0.0001)
        #plt.axis("equal")
        fig.clf()
        timec1 = time.time()
        dt = timec1 - timec
        #print(dt)
         
        
        

if __name__ == '__main__':
    main()








def detect_obs_2(scan):
    n_obj = 0
    index_obj = 0
    obj_arr = np.zeros_like(scan) # 0 for no object, 1, 2, 3, i for i_th object

    r_obs = [] # distance of objects
    th_obs = [] # angle of object
    X_obs = [] # x coordinate of object
    Y_obs = [] # y coordinate of object

    d_min = 0.20 # min distance to detect object
    d_max = 1.0 # maximum distance upto which object is to be detected
    
    d_obs = scan[0]
    obs_present = (d_obs>d_min) and (d_obs<=d_max)
    if(obs_present):
        index_obj = index_obj + 1
        obj_arr[0] = index_obj
    else:
        obj_arr[0] = 0


    for i in range (1,size(scan)):
        d_obs = scan[i]
        obs_present = (d_obs>d_min) and (d_obs<=d_max)
        if(obs_present):
            if(obj_arr[i-1]==0):
                index_obj = index_obj + 1
                obj_arr[i] = index_obj
        else:
            obj_arr[i] = 0

    return obj_arr
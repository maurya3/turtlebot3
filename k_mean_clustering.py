from cv2 import kmeans
from numpy.core.fromnumeric import shape, size
from sklearn import cluster
import rospy
from sensor_msgs.msg import LaserScan
import math
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
import matplotlib
matplotlib.use('TkAgg')
import time
import numpy as np

scan= []
R = []

def laser_cb(data):
    global scan
    scan = data.ranges
   
rospy.init_node('data_laser',anonymous=True)
sub = rospy.Subscriber('/scan',LaserScan,laser_cb)
rospy.Rate(100)


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
        if((d>0.05) and (d<=1.0)):
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

 
def elbo_checker(xobs,yobs):
    distortions = []
    slop = []
    R = np.array((xobs,yobs),dtype='float64')
    # print(R)
    for i in range(1, 11):
        km = KMeans(n_clusters=i, 
                    init='k-means++', 
                    n_init=10, 
                    max_iter=300, 
                    random_state=0)
        km.fit(R.T)
        distortions.append(km.inertia_)
     
    plt.plot(range(1, 11), distortions, marker='o')
    distortions = np.array(distortions)
    print(shape(distortions))
    
    
    for r in range(0,9):
        dy = distortions[r+1] -distortions[r]
        th = math.atan2(dy,1)

        slop.append(th*180/math.pi)
        r = r+1

    return slop


fig = plt.figure(figsize=(5,5))
fig.canvas.draw()

def main():
    while not rospy.is_shutdown():
        timec = time.time()
        X = 0
        Y = 0
        X,Y = _scan(scan)
        plt.clf()
        r_obs, th_obs, X_obs, Y_obs = detect_obs(scan)

        plt.plot([0,3],[0, 0],"g-") 

       
        #plt.plot([X, np.zeros(np.size(X))],[Y, np.zeros(np.size(Y))],"b--")
        plt.plot([X_obs, np.zeros(np.size(X_obs))],[Y_obs, np.zeros(np.size(Y_obs))],"r-")

        try:
            D = np.array((X_obs,Y_obs),dtype='float64')
            D = D.T
            slop = elbo_checker(X_obs,Y_obs)
            print(slop)
           
            km = KMeans(n_clusters=5, init='random', n_init=10, max_iter=300,tol=1e-04,random_state=0)
                
            y_km = km.fit_predict(D)

            print('chuza')
            """
            Now you plot our results - cluseters and centroids
            # """
            # plt.figure(figsize=(7, 7))
            plt.scatter(D[y_km == 0, 0], D[y_km == 0, 1], s=50, c='lightgreen',marker='s', edgecolor='black', label='Cluster 1')
            plt.scatter(D[y_km == 1, 0], D[y_km == 1, 1], s=50, c='orange',marker='o', edgecolor='black',label='Cluster 2')
            plt.scatter(D[y_km == 2, 0], D[y_km == 2, 1], s=50, c='lightblue', marker='v', edgecolor='black', label='Cluster 3')
            plt.scatter(D[y_km == 3, 0], D[y_km == 3, 1], s=50, c='orange',marker='>', edgecolor='black',label='Cluster 4')
            plt.scatter(D[y_km == 4, 0], D[y_km == 4, 1], s=50, c='lightblue', marker='<', edgecolor='black', label='Cluster 5')

            plt.scatter(km.cluster_centers_[:, 0], km.cluster_centers_[:, 1], s=250, marker='*', c='red', edgecolor='black',label='Centroid')
            plt.legend(scatterpoints=1)
                
        except:
            pass

        xylim = 8
        plt.xlim(-xylim,xylim)
        plt.ylim(-xylim,xylim)
        plt.grid(True)
        plt.pause(0.0001)
        plt.axis("equal")
        fig.clf()
        timec1 = time.time()
        dt = timec1 - timec
        #print(dt)
         
 


        

if __name__ == '__main__':
    main()




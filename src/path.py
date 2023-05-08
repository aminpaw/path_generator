#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import time
import rospy
#from tqdm import tqdm
from scipy import interpolate
from std_msgs.msg import Float64MultiArray, Float64
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from tf2_geometry_msgs import PoseStamped

s = Path


# generate a random circular track
def generate_track(n = 100):
    final=2*np.pi-np.pi/6
    # generate random angles
    theta = np.arange(0, final, final/n)
    # generate random radii
    r = np.random.uniform(0, 100, n)
    #print(r)
    x=[0]
    y=[0]
    for i in range(n):
        x.append(r[i]*np.cos(theta[i]))
        y.append(r[i]*np.sin(theta[i]))
    x=np.array(x)
    y=np.array(y)
    return [x, y]

# fit spline to track
def fit_spline(x, y, mult=3,per=1):
    #print(x,y)
    n=mult*len(x)
    # fit spline
    tck, u = interpolate.splprep([x, y], s=0,per=per)
    u_new = np.linspace(u.min(), u.max(), n)
    x, y = interpolate.splev(u_new, tck)

    return [x, y]

# smooth the track
def smooth_track(x, y, iter=2):
    # smooth the track
    for i in range(iter):
        x[1:-1] = (x[0:-2] + x[1:-1] + x[2:]) / 3
        y[1:-1] = (y[0:-2] + y[1:-1] + y[2:]) / 3
    
    return [x, y]

# calculate the track length
def calculate_track_length(x, y):
    # track length
    l=0
    n=len(x)
    #print👎
    for i in range(-1,n-1):
        try:
            l+=np.sqrt((x[i]-x[i+1])*2+(y[i]-y[i+1])*2)
        except:
            print(i,l)
    return l

# generate cones from track
def generate_cones(x, y, track_width=1,distance_between_cones=1):
    n=len(x)
    # track length
    l=calculate_track_length(x, y)
    print("Track Length: ",l)#,", Number of cones: ",n_cones)
    # track normals
    nx = np.zeros(n)
    ny = np.zeros(n)
    for i in range(n):
        if i == 0:
            nx[i] = -(y[i+1] - y[i])
            ny[i] = x[i+1] - x[i]
        elif i == n-1:
            nx[i] = -(y[i] - y[i-1])
            ny[i] = x[i] - x[i-1]
        else:
            nx[i] = -(y[i+1] - y[i-1])
            ny[i] = x[i+1] - x[i-1]
    # normalize
    norm = np.sqrt(nx*2 + ny*2)
    nx = nx / norm
    ny = ny / norm
    left_track_x = x + track_width*nx
    left_track_y = y + track_width*ny
    left_l=calculate_track_length(left_track_x, left_track_y)
    number_of_left_cones=int(left_l/distance_between_cones)
    x0,y0=left_track_x[0],left_track_y[0]
    orange_x_list=[x0]
    orange_y_list=[y0]
    left_x_list=[x0]
    left_y_list=[y0]
    left_nx_list=[nx[0]]
    left_ny_list=[ny[0]]
    for i in range(n):
        dist=np.sqrt((left_track_x[i]-x0)*2+(left_track_y[i]-y0)*2)
        if dist>=distance_between_cones:
            left_x_list.append(left_track_x[i])
            left_y_list.append(left_track_y[i])
            left_nx_list.append(nx[i])
            left_ny_list.append(ny[i])
            x0,y0=left_track_x[i],left_track_y[i]
    

    right_track_x = x - track_width*nx
    right_track_y = y - track_width*ny
    right_l=calculate_track_length(right_track_x, right_track_y)
    number_of_right_cones=int(right_l/distance_between_cones)
    x0,y0=right_track_x[0],right_track_y[0]
    orange_x_list.append(x0)
    orange_y_list.append(y0)
    right_x_list=[x0]
    right_y_list=[y0]
    right_nx_list=[nx[0]]
    right_ny_list=[ny[0]]
    for i in range(n):
        dist=np.sqrt((right_track_x[i]-x0)*2+(right_track_y[i]-y0)*2)
        if dist>=distance_between_cones:
            right_x_list.append(right_track_x[i])
            right_y_list.append(right_track_y[i])
            right_nx_list.append(nx[i])
            right_ny_list.append(ny[i])
            x0,y0=right_track_x[i],right_track_y[i]
    print("Number of left cones: ",number_of_left_cones,", Number of right cones: ",number_of_right_cones)
    return {'blue':[left_x_list, left_y_list],'yellow':[right_x_list, right_y_list],'orange':[orange_x_list, orange_y_list]}

# plot the track
def plot_track(x, y, color='black'):
    plt.plot(x, y, '-', color=color)

# plot the cones
def plot_cones(x, y, color='blue'):
    plt.plot(x, y, 'o', color=color)

# plot the track and cones
def plot_(track=None, cones=None):
    if all(v is None for v in [track, cones]):
        print("No data to plot")
        return
    plt.figure()
    plot_track(track[0], track[1]) if track is not None else None
    if cones is not None:
        for i in cones:
            plot_cones(cones[i][0], cones[i][1], color=i)
    
    plt.xlabel('x')
    plt.ylabel('y')
    plt.axis('equal')
    plt.show()

def numpyToPath(pathArray) -> Path:
    """
    Converts a numpy array to a path message
    and transforms the coordinates to the world frame

    Parameters
    ----------
    pathArray: np.ndarray, shape=(n, 2)
        Numpy array with x and y coordinates of the path

    Returns
    -------
    path: Path
        Path message
    """
    print(pathArray.shape[0])
    path = Path()
    for i in range(pathArray.shape[0]):
        pose = PoseStamped()
        pose.pose.position.x = pathArray[i,0]
        pose.pose.position.y = pathArray[i,1]
        pose.header.frame_id = path.header.frame_id = "map"
        pose.header.stamp = path.header.stamp = rospy.Time.now()
        path.poses.append(pose)
    return path

track=generate_track(50)
smoothed=smooth_track(*track,10)
spline=fit_spline(*smoothed,20)
plot_(track=spline)
print(len(spline[0]))
x_coord = spline[0]
y_coord = spline[1]
#spline [array of x, array of y]
rospy.init_node('waypointsgen', anonymous=True)
rate = rospy.Rate(0.2) # 10hz
genPath = np.array([x_coord, y_coord]).T
csv_data_temp = np.loadtxt(
    "/home/amin/FSAI-ASURT/src/path_generator/src/berlin_2018.csv",
    comments="#",
    delimiter=",",
    )
newtrack = csv_data_temp
if __name__ == '__main__':
    path_pub = rospy.Publisher('waypoints', Path, queue_size=10)
    rospy.loginfo("Waypoints generator started")
    # rate.sleep()
    '''
    store generated path in a ros float64multiarray message
    '''
    path = Path()
    path = numpyToPath(newtrack)
    rospy.loginfo(path)
    rospy.loginfo("The track is complete")
    while not rospy.is_shutdown():
        path_pub.publish(path)
        rospy.loginfo("Waypoints published")
        rate.sleep()


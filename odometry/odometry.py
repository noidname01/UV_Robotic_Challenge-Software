# In the output txt file (odometry_path.txt),
# order: map_array, origin coordinates,  array size
# 2 means the grid has been visited
# 1 means the grid has been cleaned but not visited
# 0 means the grid hasn't been visited or cleaned

import numpy as np
import cv2
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

# set initial map info
unit_width = 2  # grid width per unit (cm)
x_size, y_size = 10, 10 # array size
odom_map = np.zeros((x_size, y_size), dtype = int)
odom_map[int(x_size*0.5)][int(y_size*0.5)] = 2 # set origin visited

# set variables for critical pts coordinate
origin_set = False                           # whether (o_lx, o_ly) has been recorded
o_lx, o_ly = None, None                 # absolute location of the origin (initial position): float
o_gx, o_gy = int(x_size*0.5), int(y_size*0.5)     # origin coordinate in odom_map: int
last_x, last_y = o_gx, o_gy             # record the previous grid's coordinates: int

# output initial odometry_path.txt
np.savetxt('odometry_path.txt', odom_map, fmt = '%d', delimiter="")
with open("odometry_path.txt", "a") as f:
    f.write(str(o_gx)+' '+str(o_gy)+'\n' )
    f.write(str(x_size)+' '+str(y_size)+'\n' )

def grid_through_line(x1,y1,x2,y2):
    """ Connect given two grids and select grids to pass through.
            input: 
                    starting grid (x1, y1) and ending grid (x2, y2) (int)
            output: 
                    selected grid coordinate list including starting grid and ending grid (list of tuples)
    """
    steep = False
    pts = []
    dx = abs(x2 - x1)
    if (x2 - x1) > 0: sx = 1
    else: sx = -1
    dy = abs(y2 - y1)
    if (y2 - y1) > 0: sy = 1
    else: sy = -1
    if dy > dx:
        steep = True
        x1, y1 = y1, x1
        dx,dy = dy,dx
        sx,sy = sy,sx
    d = (2 * dy) - dx
    for i in range(0, dx):
        if steep: 
            pts.append((y1, x1))
        else: 
            pts.append((x1, y1))
        while d >= 0:
            y1 += sy
            d -= (2 * dx)
        x1 += sx
        d += (2 * dy)
    pts.append((x2,y2))
    return pts

def gen_img(file_name, arr):
    """ Generate image from given array.
            input: 
                    file_name: output jpg's name or location
                    arr:  np.array, source array
            output:
                    None
    """
    odometry_map = 255*arr
    cv2.imwrite (file_name, odometry_map) 
    
def odom_path(x, y):
    """Update odometry_path.txt, connecting the new pt and the previous one.
            input:
                    newly visited point coordinate x, y (float, float)
            output: 
                    None
    """
    global o_lx, o_ly, o_gx, o_gy, origin_set, last_x, last_y,  x_size, y_size, odom_map
    if origin_set:
        gx, gy = int(o_gx + ((x - o_lx)*100)//2), int(o_gy + ((y - o_ly)*100)//2)       # present grid coordinate in the array
        #print(gx, gy)
        # check if the data would overflow (i.e.  array is too small)
        if -1 < gx < x_size:
            pass
        elif gx < 0:
            odom_map = np.pad(odom_map, ((-gx,0),(0,0)),'constant',constant_values = (0,0))
            o_gx,   last_x,    gx  =   o_gx - gx,   last_x - gx,     0 
        else:
            odom_map = np.pad(odom_map, ((0, gx + 1 - x_size),(0,0)),'constant',constant_values = (0,0))

        if -1 < gy < y_size:
            pass
        elif gy < 0:
            odom_map = np.pad(odom_map, ((0,0),(-gy,0)),'constant',constant_values = (0,0))
            o_gy,   last_y,    gy  =   o_gy - gy,   last_y - gy,     0 
        else:
            odom_map = np.pad(odom_map, ((0, 0),(0,gy + 1 - y_size)),'constant',constant_values = (0,0))
        # update odometry path
        for (x_p, y_p) in grid_through_line(last_x, last_y, gx, gy):
            odom_map[x_p][y_p] = 2
        x_size, y_size = len(odom_map), len(odom_map[0])
        np.savetxt('odometry_path.txt', odom_map, fmt = '%d', delimiter="")
        with open("odometry_path.txt", "a") as f:
            f.write(str(o_gx)+' '+str(o_gy)+'\n' )
            f.write(str(x_size)+' '+str(y_size)+'\n' )
        gen_img('odom.jpg', odom_map)
        last_x, last_y = gx, gy
        
    else:
        o_lx, o_ly = x, y
        origin_set = True
        gen_img('odom.jpg', odom_map)

def callback(data):
    """ Callback function, executed whenever odometry node receives new data.
            print(x,y), update two odometry file, then count visited grids.
            input:
                    data ("/rtabmap/localization_pose")
            output:
                    None
    """
    print ('x:', data.pose.pose.position.x)
    print ('y:', data.pose.pose.position.y)
    odom_path(data.pose.pose.position.x, data.pose.pose.position.y)
    print(np.count_nonzero(odom_map)) 

def odom_odom():
    """  Main. 
            Create a new node 'odometry',
            Subscribe topic "/rtabmap/localization_pose"
    """
    rospy.init_node('odometry', anonymous = True)
    print('ready')
    rospy.Subscriber("/rtabmap/localization_pose", PoseWithCovarianceStamped, callback)
    rospy.spin() # I refuse to leave.

def test():
    """ for testing."""
    x = input('x: ')
    y = input('y: ')
    odom_path(x, y)
    print (x_size, y_size)
    print(np.count_nonzero(odom_map))

if __name__ == "__main__":
    odom_odom()

import numpy as np
import cv2
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

origin_set = False      # for odom(x,y)
origin_set2 = False    # for odm_path(x,y)
# grid
last_x = 0 
last_y = 0 
# real coordinate
origin_x = None 
origin_y = None

# set map size (even int)
# each grid width 2(cm)
x_size = 1000
y_size = 1000
odometry_map = np.full((x_size, y_size), False)  # pts only
odometry_map2 = np.full((x_size, y_size), False) # containing path

def odom(x, y):
    """ Update odometry_pt.txt, containing only discrete pts.
            input:    
                    x, y (float) newly visited point  coordinate
            output: 
                    None
    """
    global origin_set, origin_x, origin_y
    if origin_set:
        x_g =int(((x-origin_x)*100) //2)
        y_g =int(((y-origin_y)*100) //2)
        odometry_map[int(x_g + 0.5*x_size)][int(y_g + 0.5*y_size)] = True
        np.savetxt('odometry_pt.txt', odometry_map, fmt = '%d', delimiter="")
        return None
    origin_x, origin_y = x, y
    origin_set = True
    odom(origin_x, origin_y)

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

def odom_path(x,y):
    """ Update odometry_path.txt, connecting the new pt and the previous one.
            input:
                    newly visited point coordinate x, y (float)
            output: 
                    None
    """
    global origin_set2, origin_x, origin_y, last_x, last_y
    if origin_set2:
        x_g =int(((x-origin_x)*100) //2)
        y_g =int(((y-origin_y)*100) //2)
        for (x_p, y_p) in grid_through_line(last_x, last_y, x_g, y_g):
            odometry_map2[int(x_p + 0.5*x_size)][int(y_p + 0.5*y_size)] = True
        np.savetxt('odometry_path.txt', odometry_map2, fmt = '%d', delimiter="")
        last_x, last_y = x_g, y_g
        return None
    origin_x, origin_y =x,  y
    origin_set2 = True
    odom_path(origin_x, origin_y)


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
    odom(data.pose.pose.position.x, data.pose.pose.position.y)
    odom_path(data.pose.pose.position.x, data.pose.pose.position.y)
    print(np.count_nonzero(odometry_map)) # only counts visited pt
    print(np.count_nonzero(odometry_map2)) # counts pt and path


def odom_odom():
    """ Setup. 
            Create a new node 'odometry',
            Subscribe topic "/rtabmap/localization_pose"
    """
    rospy.init_node('odometry', anonymous = True)
    print('ready')
    rospy.Subscriber("/rtabmap/localization_pose", PoseWithCovarianceStamped, callback)
    rospy.spin() # I refuse to leave.

def gen_img(data):
    
    odom_map = np.genfromtxt('odometry_path.txt', dtype=float)
    odom_map = 255*odom_map
    cv2.imwrite('../uv-robotic-web/src/image/odom.jpg',odom_map) #temporarily, need to change to build directory
    
if __name__ == "__main__":
    odom_odom()
    

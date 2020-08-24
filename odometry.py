import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

origin_set = False
origin_set2 = False
last_x = 0 # grid
last_y = 0 
origin_x = None # real coordinate
origin_y = None

# set map size (even int)
# each grid width 2(cm)
x_size = 100
y_size = 100
odometry_map = np.full((x_size, y_size), False)
odometry_map2 = np.full((x_size, y_size), False)
print(odometry_map)


def odom(x, y):
    """ Update the odometry matrix.
            input: newly visited point  
    """
    global origin_set, origin_x, origin_y
    if origin_set:
        x_g =int(((x-origin_x)*100) //2)
        y_g =int(((y-origin_y)*100) //2)
        odometry_map[int(x_g + 0.5*x_size)][int(y_g + 0.5*y_size)] = True
        np.savetxt('odometry_pt.txt', odometry_map, fmt = '%d')
        return None
    origin_x, origin_y = x, y
    origin_set = True
    odom(origin_x, origin_y)

def grid_through_line(x1,y1,x2,y2):
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
    """ Update the odometry matrix, connecting the new pt and the previous one.
            input: newly visited point 
    """
    global origin_set2, origin_x, origin_y, last_x, last_y
    if origin_set2:
        x_g =int(((x-origin_x)*100) //2)
        y_g =int(((y-origin_y)*100) //2)
        for (x_p, y_p) in grid_through_line(last_x, last_y, x_g, y_g):
            odometry_map2[int(x_p + 0.5*x_size)][int(y_p + 0.5*y_size)] = True
        np.savetxt('odometry_path.txt', odometry_map2, fmt = '%d')
        last_x, last_y = x_g, y_g
        return None
    origin_x, origin_y =x,  y
    origin_set2 = True
    odom_path(origin_x, origin_y)


def callback(data):
    print 'x:', data.pose.pose.position.x
    print 'y:', data.pose.pose.position.y
    odom(data.pose.pose.position.x, data.pose.pose.position.y)
    odom_path(data.pose.pose.position.x, data.pose.pose.position.y)
    print(np.count_nonzero(odometry_map))
    print(np.count_nonzero(odometry_map2))


def catch_odom():
    rospy.init_node('odometry', anonymous = True)
    print('ready')
    rospy.Subscriber("/rtabmap/localization_pose", PoseWithCovarianceStamped, callback)
    rospy.spin()

if __name__ == "__main__":
    catch_odom()
    
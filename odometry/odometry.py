# In the output txt file,
# 2 means the grid has been visited
# 1 means the grid has been cleaned but not visited
# 0 means the grid hasn't been visited or cleaned

import numpy as np
import cv2
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

# set initial map info
unit_width = 2  # grid width per unit (cm)
x_size, y_size = 15, 15 # array size, size should be bigger than 2*unit_width+1 !
odom_map1 = np.zeros((x_size, y_size), dtype = np.uint8) # odom_map for cleaned
odom_map1 = cv2.line(odom_map1, (int(y_size*0.5), int(x_size*0.5)), (int(y_size*0.5), int(x_size*0.5)), 1, 2*unit_width) # set origin cleaned
odom_map2 = np.zeros((x_size, y_size), dtype = np.uint8) # odom_map for visited
odom_map2[int(x_size*0.5)][int(y_size*0.5)] = 1 # set origin visited
odom_map = odom_map1 + odom_map2

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

def gen_img(file_name, arr):
    """ Generate image from given array.
            input: 
                    file_name: output jpg's name or location
                    arr:  np.array, source array
            output:
                    None
    """
    odometry_map = 127*arr
    cv2.imwrite (file_name, odometry_map) 
    
def odom_path(x, y):
    """Update odometry_path.txt, connecting the new pt and the previous one.
            input:
                    newly visited point coordinate x, y (float, float)
            output: 
                    None
    """
    global o_lx, o_ly, o_gx, o_gy, origin_set, last_x, last_y,  x_size, y_size, odom_map1, odom_map2, odom_map
    if origin_set:
        gx, gy = int(o_gx + ((x - o_lx)*100)//2), int(o_gy + ((y - o_ly)*100)//2)       # present grid coordinate in the array
        #print(gx, gy)
        # check if the data would overflow (i.e.  array is too small)
        if -1 < (gx-unit_width) and (gx+unit_width) < x_size:
            pass
        elif (gx-unit_width) < 0:
            odom_map1 = np.pad(odom_map1, ((-gx+unit_width,0),(0,0)),'constant',constant_values = (0,0))
            odom_map2 = np.pad(odom_map2, ((-gx+unit_width,0),(0,0)),'constant',constant_values = (0,0))
            o_gx,   last_x,    gx  =   o_gx - gx + unit_width,   last_x - gx + unit_width,     unit_width
        else:
            odom_map1 = np.pad(odom_map1, ((0, gx + unit_width + 1 - x_size),(0,0)),'constant',constant_values = (0,0))
            odom_map2 = np.pad(odom_map2, ((0, gx + unit_width + 1 - x_size),(0,0)),'constant',constant_values = (0,0))

        if -1 < (gy-unit_width) and (gy+unit_width) < y_size:
            pass
        elif (gy-unit_width) < 0:
            odom_map1 = np.pad(odom_map1, ((0,-gy+unit_width),(0,0)),'constant',constant_values = (0,0))
            odom_map2 = np.pad(odom_map2, ((0,-gy+unit_width),(0,0)),'constant',constant_values = (0,0))
            o_gy,   last_y,    gy  =   o_gy - gy + unit_width,   last_y - gy + unit_width,     unit_width
        else:
            odom_map1 = np.pad(odom_map1, ((0, 0),(0,gy + unit_width + 1 - y_size)),'constant',constant_values = (0,0))
            odom_map2 = np.pad(odom_map2, ((0, 0),(0,gy + unit_width + 1 - y_size)),'constant',constant_values = (0,0))

        # update odometry path
        odom_map1 = cv2.line(odom_map1,(last_y, last_x), (gy, gx), 1, 2*unit_width)
        odom_map2 = cv2.line(odom_map2,(last_y, last_x), (gy, gx), 1)
        odom_map = odom_map1 + odom_map2
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
    rospy.init_node('UVbot_odometry', anonymous = True)
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

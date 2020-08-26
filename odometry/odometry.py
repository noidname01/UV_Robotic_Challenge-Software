# In the output txt file,
# 2 means the grid has been visited
# 1 means the grid has been sterilized but not visited
# 0 means the grid hasn't been visited or sterilized

import numpy as np
import cv2
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

# set initial map info
unit_width = 2  # grid width per unit (cm)
w_sterilized = 50 # maximum effective sterilization range (unit grids)
x_size, y_size = 150, 150 # array size, size should be bigger than 2*w_sterilized+1 !
odom_map1 = np.zeros((x_size, y_size), dtype = np.uint8) # odom_map for sterilized
odom_map1 = cv2.circle(odom_map1, (int(y_size*0.5), int(x_size*0.5)), w_sterilized, 1, -1) # set origin sterilized
odom_map2 = np.zeros((x_size, y_size), dtype = np.uint8) # odom_map for visited
odom_map2[int(x_size*0.5)][int(y_size*0.5)] = 1 # set origin visited
odom_map = odom_map1 + odom_map2

# set variables for critical pts coordinate
origin_set = False                           # whether (o_lx, o_ly) has been recorded or not
o_lx, o_ly = None, None                 # absolute location of the origin (initial position): float
o_gx, o_gy = int(x_size*0.5), int(y_size*0.5)     # origin coordinate in odom_map: int
last_x, last_y = o_gx, o_gy             # record the previous grid's coordinates: int

# output initial odometry_path.txt
np.savetxt('odometry_path.txt', odom_map, fmt = '%d', delimiter="")  # write odometry array into file
with open("odometry_path.txt", "a") as f:
    f.write(str(o_gx)+' '+str(o_gy)+'\n' )          # write coordinates of origin into file (x,y)
    f.write(str(x_size)+' '+str(y_size)+'\n' )     # write sizes of the odometry into file (x_size, y_size)

def gen_img(file_name, arr):
    """ Generate image from given array.
            visited points = white (254), sterilized points = gray (127), others = black (0)
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
        gx, gy = int(o_gx + ((x - o_lx)*100)//unit_width), int(o_gy + ((y - o_ly)*100)//unit_width)       # present grid coordinate in the array
        #print(gx, gy)
        # check if the data would overflow (i.e.  array is too small)
        # modifying x-axis
        if -1 < (gx-w_sterilized) and (gx+w_sterilized) < x_size:
            pass
        elif (gx-w_sterilized) < 0:
            odom_map1 = np.pad(odom_map1, ((-gx+w_sterilized,0),(0,0)),'constant',constant_values = (0,0))
            odom_map2 = np.pad(odom_map2, ((-gx+w_sterilized,0),(0,0)),'constant',constant_values = (0,0))
            o_gx,   last_x,    gx  =   o_gx - gx + w_sterilized,   last_x - gx + w_sterilized,     w_sterilized
        else:
            odom_map1 = np.pad(odom_map1, ((0, gx + w_sterilized + 1 - x_size),(0,0)),'constant',constant_values = (0,0))
            odom_map2 = np.pad(odom_map2, ((0, gx + w_sterilized + 1 - x_size),(0,0)),'constant',constant_values = (0,0))
        # modifying y-axis
        if -1 < (gy-w_sterilized) and (gy+w_sterilized) < y_size:
            pass
        elif (gy-w_sterilized) < 0:
            odom_map1 = np.pad(odom_map1, ((0,0),(-gy+w_sterilized,0)),'constant',constant_values = (0,0))
            odom_map2 = np.pad(odom_map2, ((0,0),(-gy+w_sterilized,0)),'constant',constant_values = (0,0))
            o_gy,   last_y,    gy  =   o_gy - gy + w_sterilized,   last_y - gy + w_sterilized,     w_sterilized
        else:
            odom_map1 = np.pad(odom_map1, ((0, 0),(0,gy + w_sterilized + 1 - y_size)),'constant',constant_values = (0,0))
            odom_map2 = np.pad(odom_map2, ((0, 0),(0,gy + w_sterilized + 1 - y_size)),'constant',constant_values = (0,0))

        # update odometry path
        odom_map1 = cv2.line(odom_map1,(last_y, last_x), (gy, gx), 1, 2*w_sterilized)
        odom_map2 = cv2.line(odom_map2,(last_y, last_x), (gy, gx), 1)
        odom_map = odom_map1 + odom_map2
        x_size, y_size = len(odom_map), len(odom_map[0])
        np.savetxt('odometry_path.txt', odom_map, fmt = '%d', delimiter="")
        with open("odometry_path.txt", "a") as f:
            f.write(str(o_gx)+' '+str(o_gy)+'\n' )
            f.write(str(x_size)+' '+str(y_size)+'\n' )
        gen_img('odom.jpg', odom_map)
        last_x, last_y = gx, gy
        
    else:   # receive first data (set as origin)
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
    print('sterilized area (m*m):', 0.00004*np.count_nonzero(odom_map)) 

def odom_odom():
    """  Main. 
            Create a new node 'UVbot_odometry'.
            Subscribe topic "/rtabmap/localization_pose".
    """
    rospy.init_node('UVbot_odometry', anonymous = True)
    print('ready')
    rospy.Subscriber("/rtabmap/localization_pose", PoseWithCovarianceStamped, callback)
    rospy.spin() 

def test():
    """ for testing."""
    x = input('x: ')
    y = input('y: ')
    odom_path(x, y)
    print (x_size, y_size)
    print(np.count_nonzero(odom_map))

if __name__ == "__main__":
    odom_odom()
    

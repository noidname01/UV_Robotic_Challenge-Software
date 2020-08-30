# Odometry

## What
Odometry is a type of data that records the **track** of our robot.

Usually it is generated from motion sensors to help correct the error caused by the motors,
but in our case, the position of our robot would be marked on the map of the room, and since the map is generated from the camera (Intel RealSense Depth Camera D435), we choose to record our localization data from the camera, too.

## Why 
Historical track is an essential and basic part for our whole project, it will be used for further manipulation.

## How 
First, we use `rospy`, the communication system in ROS, to capture the localization data message published from the camera.
``` python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

rospy.init_node('UVbot_odometry', anonymous = True)
rospy.Subscriber("/rtabmap/localization_pose", PoseWithCovarianceStamped, callback)
rospy.spin()
```

Second, we process our data whenever recieved one, using `odom_path(data.pose.pose.position.x, data.pose.pose.position.y)` written in `callback(data)` function.


Now, let us take a look at `odom_path()` function.

This function generates an numpy array conprised of 0,1 and 2, where  
'2' represents that this grid has been visited;  
'1' for sterilized but not visited;  
'0' the rest.

Since we are running our programs on Raspberry Pi 4 rather than normal laptops or computers, our goal is to reduce data transmission and avoid using large-sized array.  
Hence, we adjust the size of array that could record the complete data using `np.pad()`.  
Below is the case for modifying x-size of array.

```Python
if -1 < (gx-w_sterilized) and (gx+w_sterilized) < x_size:
            pass
elif (gx-w_sterilized) < 0:
    odom_map1 = np.pad(odom_map1, ((-gx+w_sterilized,0),(0,0)),'constant',constant_values = (0,0))
    odom_map2 = np.pad(odom_map2, ((-gx+w_sterilized,0),(0,0)),'constant',constant_values = (0,0))
    o_gx, last_x,  gx = (o_gx - gx + w_sterilized),  (last_x - gx + w_sterilized),  w_sterilized
else:
    odom_map1 = np.pad(odom_map1, ((0, gx + w_sterilized + 1 - x_size),(0,0)),'constant',constant_values = (0,0))
    odom_map2 = np.pad(odom_map2, ((0, gx + w_sterilized + 1 - x_size),(0,0)),'constant',constant_values = (0,0))

```

The data we receive are discrete points. Since our robot does not move so fast and its track would not be curve, it is reasonable to assume that the real trajectory is the connection of two points, we can then update the sterilized-map and visited-map by applying `cv2.line()`.

Three `numpy.arrays` are created, one records sterilized area, another for trajectory, and the other adds them up and overlaps the information.

```Python
odom_map1 = cv2.line(odom_map1,(last_y, last_x), (gy, gx), 1, 2*w_sterilized)
odom_map2 = cv2.line(odom_map2,(last_y, last_x), (gy, gx), 1)
odom_map = odom_map1 + odom_map2
```

Finally, output the txt file including odom_map array,the coordinates of origin point, map size, and present coordinates for further manipulation using `np.savetxt()` and `file.write()`.

By the way, we have a `test()` function that can manually enter coordinates and output jpg image file using `cv2.imwrite()`, which is for testing only (image including more information would be generated in other programs).

The following is the screenshot of the testing `odom_path.txt` and `odom.jpg`.

![screenshot](./odometry/screenshot.png)


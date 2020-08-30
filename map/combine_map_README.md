# Combine_map

## What
This program receives the information from **odometry.py** and **obstacle.py**, and output the **integrated** text file and image file, both showing each grid's present state: obstacle(e.g. the wall), visited, sterilized, or unsterilized.

## Why
The integrated information is critical because our goal is to clean the entire room and avoid repeated or redundant route. By updating and classifying each grid's status, we can achieve **dynamic route planning**, also **visualize the track and sterizing progress** for users to check in **real time**.


## How
We first create a node for launching program in  `ROS`, and run the main `combine_node()` function.
```python
def combine_map_node():
    rospy.init_node('UVbot_combine_map', anonymous=True)
    while not rospy.is_shutdown():
        combine_map()
```
`combine_map()` generates a numpy array conprised of 0, 1, 2 and 3, where  
'3' indicates obstacles;   
'2' represents that this grid has been visited;  
'1' for disinfected but not visited;  
'0' the rest. 

Then it will output a text file including the array, related information, and a jpg file according to the array, where  
black ([0, 0, 0]) represents obstacles;  
yellow ([0, 255, 255]) represents path;  
purple ([255, 0, 0]) for region that have been disinfected;  
white ([255, 255, 255]) for region that haven't been disinfected.

Following is how we achieved it.

First, read the text files generated from odometry.py and obstacle.py, then store the data we need respectively: 
1. the map array 
2. the origin (initial point) coordinates in the array
3. x and y size of the array
4. the present coordinate in the array

Since the data transmitted is the minimum array, the size and coordinates of the two arrays might not be identical.  
Thus, to integrate the arrays, we overlap the origin points and adjust the size of the array.

```python
    combine_range_x = max(obstacle_origin_x, odometry_origin_x) + max(obstacle_range_x-obstacle_origin_x, odometry_range_x-odometry_origin_x)
    combine_range_y = max(obstacle_origin_y, odometry_origin_y) + max(obstacle_range_y-obstacle_origin_y, odometry_range_y-odometry_origin_y)
    combine_origin_x = max(obstacle_origin_x, odometry_origin_x)
    combine_origin_y = max(obstacle_origin_y, odometry_origin_y)
    combine = [[] for i in range(combine_range_x)]
    for i in range(combine_range_x):
        for j in range(combine_range_y):
            combine[i].append(0)
```
After getting an array with proper size and coordinate system, we can fill in the value for each grid with some translation.

```python
    for i in range(obstacle_range_x):
        for j in range(obstacle_range_y):
            if obstacle[i][j] == 3:
                combine[i+combine_origin_x-obstacle_origin_x][j+combine_origin_y-obstacle_origin_y] = 3
    for i in range(odometry_range_x):
        for j in range(odometry_range_y):
            if odometry[i][j] == 1 or odometry[i][j] == 2:
                combine[i+combine_origin_x-odometry_origin_x][j+combine_origin_y-odometry_origin_y] = odometry[i][j]
```
At last, generate the text file and jpg file using `np.savetxt()`, `file.write()` and `cv2.imwrite()`.  
Below is an example of combime_map.jpg

![combine_map.jpg](https://github.com/noidname01/UV_Robotic_Challenge-Software/blob/master/map/combine_map.jpg)



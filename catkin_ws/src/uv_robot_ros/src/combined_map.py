#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import os

def combine_map_node():
    '''
    description:
        Create a node and run the function "combine_map()"
    input:
        None
    output:
        None
    '''
    rospy.init_node('UVbot_combine_map', anonymous=True)
    while not rospy.is_shutdown():
        combine_map()

def combine_map():
    '''
    description:
        Construct a file 'combine_map.txt' to represent the overall map. 
        In "combine_map.txt", there is an x*y array, which means the room is divided by x*y spaces. Each space is a 2cm*2cm squares. Following the array are the origin location of the robot and the size of the map.
        First, read the two files "obstacle_smooth_fill.txt" and "odometry_path.txt". Decide a minimal rectangle that can contain the two map, and stack the two map on it together. The origin location of the two map should be the same place. Finally, write the new map to the file "combine_map.txt".
    input:
        None
    output:
        None
    '''
# read obstacle
    with open('obstacle_smooth_fill.txt', 'r') as f:
        lines = [line.strip() for line in f.readlines()]
    obstacle_range_x = int(lines[-1].split()[0])
    obstacle_range_y = int(lines[-1].split()[1])
    obstacle_origin_x = int(lines[-2].split()[0])
    obstacle_origin_y = int(lines[-2].split()[1])
    obstacle = [[] for i in range(obstacle_range_x)]
    for i in range(obstacle_range_x):
        for j in range(obstacle_range_y):
            obstacle[i].append(int(lines[i][j]))

# read odometry
    with open('odometry_path.txt', 'r') as f:
        lines = [line.strip() for line in f.readlines()]
    odometry_range_x = int(lines[-2].split()[0])
    odometry_range_y = int(lines[-2].split()[1])
    odometry_origin_x = int(lines[-3].split()[0])
    odometry_origin_y = int(lines[-3].split()[1])
    now_loc_x = int(lines[-1].split()[0])
    now_loc_y = int(lines[-1].split()[1])
    odometry = [[] for i in range(odometry_range_x)]
    for i in range(odometry_range_x):
        for j in range(odometry_range_y):
            odometry[i].append(int(lines[i][j]))

# determine size and coordinate system
    combine_range_x = max(obstacle_origin_x, odometry_origin_x) + max(obstacle_range_x-obstacle_origin_x, odometry_range_x-odometry_origin_x)
    combine_range_y = max(obstacle_origin_y, odometry_origin_y) + max(obstacle_range_y-obstacle_origin_y, odometry_range_y-odometry_origin_y)
    combine_origin_x = max(obstacle_origin_x, odometry_origin_x)
    combine_origin_y = max(obstacle_origin_y, odometry_origin_y)
    
# for txt file
    combine_to_txt = np.zeros((combine_range_x, combine_range_y), dtype = np.uint8)
    for i in range(odometry_range_x):
        for j in range(odometry_range_y):
            if odometry[i][j] == 1 or odometry[i][j] == 2:
                combine_to_txt[i+combine_origin_x-odometry_origin_x][j+combine_origin_y-odometry_origin_y] = odometry[i][j]
    for i in range(obstacle_range_x):
        for j in range(obstacle_range_y):
            if obstacle[i][j] == 3:
                combine_to_txt[i+combine_origin_x-obstacle_origin_x][j+combine_origin_y-obstacle_origin_y] = 3
    np.savetxt('combine_map.txt', combine_to_txt, fmt = '%d', delimiter="")
    with open('combine_map.txt', 'a') as f:
        f.write(str(combine_origin_x)+' '+str(combine_origin_y)+'\n')
        f.write(str(combine_range_x)+' '+str(combine_range_y)+'\n')
        f.write(str(now_loc_x)+' '+str(now_loc_y))
    
# for jpg file
    combine = np.full((combine_range_x, combine_range_y, 3), 255 ,dtype = np.uint8)
    # white = [255, 255, 255], haven't been disinfected (0)
    black = [0, 0, 0]       # wall(3)
    yellow = [0, 255, 255]  # path(2)
    # purple = [255, 0, 0], disinfected(1), transparent on image
    # array without disinfected area
    for i in range(obstacle_range_x):
        for j in range(obstacle_range_y):
            if obstacle[i][j] == 3:
                combine[i+combine_origin_x-obstacle_origin_x][j+combine_origin_y-obstacle_origin_y] = black
    for i in range(odometry_range_x):
        for j in range(odometry_range_y):
            if odometry[i][j] == 2:
                combine[i+combine_origin_x-odometry_origin_x][j+combine_origin_y-odometry_origin_y] = yellow
    # array for disinfected area (transparent)
    for i in range(odometry_range_x):
        for j in range(odometry_range_y):
            if odometry[i][j] == 1:
                x = i+combine_origin_x-odometry_origin_x
                y = j+combine_origin_y-odometry_origin_y
                combine[x][y] = np.floor(combine[x][y]*0.7) + [76, 0, 0]  # 0.7*origin + 0.3*[255,0,0]
    cv2.imwrite(os.getcwd()+'/combine_map.jpg', combine)

combine_map_node()

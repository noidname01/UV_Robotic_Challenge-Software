#!/usr/bin/env python
import rospy

def combine_map_node():
    rospy.init_node('UVbot_combine_map', anonymous=True)
    while not rospy.is_shutdown():
        combine_map()

def combine_map():
    '''
    Construct a file 'combine_map.txt' to represent the overall map. 
    In "combine_map.txt", there is an x*y array, which means the room is divided by x*y spaces. Each space is a 2cm*2cm squares. Following the array are the origin location of the robot and the size of the map.
    First, read the two files "obstacle_smooth_fill.txt" and "odometry_path.txt". Decide a minimal rectangle that can contain the two map, and stack the two map on it together. The origin location of the two map should be the same place. Finally, write the new map to the file "combine_map.txt".
    '''
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

    combine_range_x = max(obstacle_origin_x, odometry_origin_x) + max(obstacle_range_x-obstacle_origin_x, odometry_range_x-odometry_origin_x)
    combine_range_y = max(obstacle_origin_y, odometry_origin_y) + max(obstacle_range_y-obstacle_origin_y, odometry_range_y-odometry_origin_y)
    combine_origin_x = max(obstacle_origin_x, odometry_origin_x)
    combine_origin_y = max(obstacle_origin_y, odometry_origin_y)
    combine = [[] for i in range(combine_range_x)]
    for i in range(combine_range_x):
        for j in range(combine_range_y):
            combine[i].append(0)
    '''
    print('obs_r_x:', obstacle_range_x)
    print('obs_r_y:', obstacle_range_y)
    print('obs_o_x:', obstacle_origin_x)
    print('obs_o_y:', obstacle_origin_y)
    print('odom_r_x:', odometry_range_x)
    print('odom_r_y:', odometry_range_y)
    print('odom_o_x:', odometry_origin_x)
    print('odom_o_y:', odometry_origin_y)
    print('com_r_x:', combine_range_x)
    print('com_r_y:', combine_range_y)
    print('com_o_x:', combine_origin_x)
    print('com_o_y:', combine_origin_y)
    '''

    for i in range(obstacle_range_x):
        for j in range(obstacle_range_y):
            if obstacle[i][j] == 3:
                combine[i+combine_origin_x-obstacle_origin_x][j+combine_origin_y-obstacle_origin_y] = 3
    for i in range(odometry_range_x):
        for j in range(odometry_range_y):
            if odometry[i][j] == 1 or odometry[i][j] == 2:
                combine[i+combine_origin_x-odometry_origin_x][j+combine_origin_y-odometry_origin_y] = odometry[i][j]

    with open('combine_map.txt', 'w') as f:
        for i in range(combine_range_x):
            for j in range(combine_range_y):
                f.write(str(combine[i][j]))
            f.write('\n')
        f.write(str(combine_origin_x)+' '+str(combine_origin_y)+'\n')
        f.write(str(combine_range_x)+' '+str(combine_range_y)+'\n')
        f.write(str(now_loc_x)+' '+str(now_loc_y))
    
    map_to_jpg()

def map_to_jpg():
    '''
    This function is to write a .jpg file. 
    Use black ([0, 0, 0]) to represent walls; 
    white ([255, 255, 255]) to represent region that haven't been disinfected; 
    yellow ([0, 255, 255)] to represent path; 
    purple ([255, 0, 0]) to represent the region that have been disinfected.
    '''
    with open('combine_map.txt', 'r') as f:
        lines = [line.strip() for line in f.readlines()]
    range_x = int(lines[-2].split()[0])
    range_y = int(lines[-2].split()[1])
    combine_map = [[] for i in range(range_x)]
    for i in range(range_x):
        for j in range(range_y):
            if int(lines[i][j]) == 0:
                combine_map[i].append([255, 255, 255])
            elif int(lines[i][j]) == 1:
                combine_map[i].append([255, 0, 0])
            elif int(lines[i][j]) == 2:
                combine_map[i].append([0, 255, 255])
            elif int(lines[i][j]) == 3:
                combine_map[i].append([0, 0, 0])

    combine_jpg = np.array(combine_map)
    cv2.imwrite('combine_map.jpg', combine_jpg)   

if __name__ == '__main__':
    while True:
        try:
             combine_map_node()
        except:
             pass

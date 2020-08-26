with open('obstacle_smooth.txt', 'r') as f:
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
odometry_range_x = int(lines[-1].split()[0])
odometry_range_y = int(lines[-1].split()[1])
odometry_origin_x = int(lines[-2].split()[0])
odometry_origin_y = int(lines[-2].split()[1])
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
        combine[i].append(1)
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


for i in range(odometry_range_x):
    for j in range(odometry_range_y):
        if odometry[i][j] == 1 or odometry[i][j] == 2:
            combine[i+combine_origin_x-odometry_origin_x][j+combine_origin_y-odometry_origin_y] = odometry[i][j]
for i in range(obstacle_range_x):
    for j in range(obstacle_range_y):
        if obstacle[i][j] == 3:
            combine[i+combine_origin_x-obstacle_origin_x][j+combine_origin_y-obstacle_origin_y] = 3

with open('combine_map.txt', 'w') as f:
    for i in range(combine_range_x):
        for j in range(combine_range_y):
            f.write(str(combine[i][j]))
        f.write('\n')
    f.write(str(combine_origin_x)+' '+str(combine_origin_y)+'\n')
    f.write(str(combine_range_x)+' '+str(combine_range_y))





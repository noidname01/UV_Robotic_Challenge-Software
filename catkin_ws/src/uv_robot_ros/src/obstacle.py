#! /usr/bin/env python
import os
import numpy as np
import cv2
import rospy


filename = None
def pcd_exist():
    '''
    Check if there is any .pcd file in the directory. If there is, let the variable 'filename' be the name of one of the pcd files.
    output:
        A bool. True if there are pcd files in the directory.
    '''
    global filename
    path = os.getcwd()
    rospy.loginfo(path)
    name_list = os.listdir(path)
    for name in name_list:
        
        try:
            rospy.loginfo(name)
            if name[-3:] == 'pcd': 
                filename = name
                print(True,name)
                return True

        except:
            pass
    print(False)
    return False

def make_obstacle_map():
    '''
    This function is to generate an obstacle map, and write it to a file 'obstacle_map'.
    First, read a .pcd file, and get the pointcloud data of the map. Because the robot is only 1.6 meters tall, we just have to care about the obstacle shorter than 1.6 meters. The file 'obstacle_tmp.txt' records the obstacle that is shorter than 1.6 meters.
    Second, read the file 'obstacle_tmp.txt' and find the range of the pointcloud. Construct an array representing the map, each of the element represented a 2cm*2cm space. The elements are denoted by the number of the points in their representing space.
    Third, for every element, count how many points are around the representing space, and add the number to its own number. If the final number is larger than a variable (thres), denote the element by '3', otherwise denote it by '0'. This step intends to smooth the map and avoids errors.
    Forth, from four sides of the map, denote elements by '3' until the element is '3' originally. This step intends to denote the space outside the wall (but not detected by the camera) to wall.
    Finally, write the array to the file 'obstacle_smooth_fill.txt'. The two lines in the bottom of the file represent "the origin of the robot" and "the size of the room".
    '''
    while pcd_exist():
        rospy.loginfo("Reading {} ...!!!!!!".format(filename))
        with open(filename, 'r') as f:
            lines = [line.strip().split() for line in f.readlines()]
        os.remove(filename)
        is_data = False
        data = []
        newlines = []
        for line in lines:
            if is_data == False:
                data.append(line)
            if line[0] == 'DATA':
                is_data = True
                continue
            if is_data == True:
                if float(line[2]) < 1.6:
                    line[2] = str(0)
                    newlines.append(line)
        count = len(newlines)
        with open('obstacle_tmp.txt', 'w') as f:
            for line in data:
                for element in line:
                    if element == 'POINTS' or element == 'WIDTH':
                        f.write(element)
                        f.write(' ')
                        f.write(str(count))
                        break
                    f.write(element)
                    f.write(' ')
                f.write('\n')
            for line in newlines:
                for element in line:
                    f.write(element)
                    f.write(' ')
                f.write('\n')
        
        thres = 5
        with open('obstacle_tmp.txt', 'r') as f:
            lines = [line.strip().split() for line in f.readlines()]
        info = []
        data = []
        is_data = False
        for line in lines:
            if is_data == False:
                info.append(line)
            if line[0] == 'DATA':
                is_data = True
                continue
            if is_data == True:
                data.append(line)

        min_x = 100
        max_x = -100
        min_y = 100
        max_y = -100
        for d in data:
            if float(d[0]) < min_x:
                min_x = float(d[0])
            if float(d[0]) > max_x:
                max_x = float(d[0])
            if float(d[1]) < min_y:
                min_y = float(d[1])
            if float(d[1]) > max_y:
                max_y = float(d[1])

        for d in data:
            d[0] = float(d[0]) - min_x
            d[1] = float(d[1]) - min_y

        total_x = (max_x - min_x) * 50
        total_y = (max_y - min_y) * 50
        range_x = int(total_x//1) + 1
        range_y = int(total_y//1) + 1
        #print(range_x, range_y)

        dot_dict = dict()
        for i in range(range_x):
            for j in range(range_y):
                dot_dict[(i, j)] = 0
        for d in data:
            dot_dict[(int(d[0]*10//1),int(d[1]*10//1))] += 1


        with open('obstacle.txt', 'w') as f:
            #f.write('P2\n')
            #f.write('15\n')
            for i in range(range_x):
                for j in range(range_y):
                    if dot_dict[(i, j)] > 0:
                        f.write(str('3'))
                    else:
                        f.write(str('0'))
                    #print(dot_dict[(i, j('0')])
                f.write('\n')
            f.write(str(int(-min_x*10//1))+' '+str(int(-min_y*10//1))+'\n')
            f.write(str(range_x)+' '+str(range_y))
#smooth
        for i in range(range_x):
            for j in range(range_y):
                count = 0
                if i>0 and j>0 and i<range_x-2 and j<range_y-2:
                    if dot_dict[((i-1), (j-1))] > thres:
                        count += 1
                    else:
                        count -= 1
                    if dot_dict[((i-1), j)] > thres:
                        count += 1
                    else:
                        count -= 1
                    if dot_dict[(i, (j-1))] > thres:
                        count += 1
                    else:
                        count -= 1
                    if dot_dict[((i+1), (j+1))] > thres:
                        count += 1
                    else:
                        count -= 1
                    if dot_dict[((i+1), j)] > thres:
                        count += 1
                    else:
                        count -= 1
                    if dot_dict[(i, (j+1))] > thres:
                        count += 1
                    else:
                        count -= 1
                    if dot_dict[((i-1), (j+1))] > thres:
                        count += 1
                    else:
                        count -= 1
                    if dot_dict[((i+1), (j-1))] > thres:
                        count += 1
                    else:
                        count -= 1
                    dot_dict[(i, j)] += count*5

                    
#/smooth

        with open('obstacle_smooth.txt', 'w') as f:
            #f.write('P2\n')
            #f.write('15\n')
            for i in range(range_x):
                for j in range(range_y):
                    if dot_dict[(i, j)] > 0:
                        f.write(str('3'))
                    else:
                        f.write(str('0'))
                    #print(dot_dict[(i, j('0')])
                f.write('\n')
            f.write(str(int(-min_x*10//1))+' '+str(int(-min_y*10//1))+'\n')
            f.write(str(range_x)+' '+str(range_y))
        
#fill
        fill_dict = dict()
        for i in range(range_x):
            for j in range(range_y):
                fill_dict[(i, j)] = dot_dict[(i, j)]
        for i in range(range_x):
            for j in range(range_y-1):
                if dot_dict[(i, j)] <= 0:
                    fill_dict[(i, j)] = 1
                    if dot_dict[(i, j+1)] > 0:
                        break
        for i in range(range_x):
            for j in reversed(range(1, range_y)):
                if dot_dict[(i, j)] <= 0:
                    fill_dict[(i, j)] = 1
                    if dot_dict[(i, j-1)] > 0:
                        break
                else:
                    break
        for j in range(range_y):
            for i in range(range_x-1):
                if dot_dict[(i, j)] <= 0:
                    fill_dict[(i, j)] = 1
                    if dot_dict[(i+1, j)] > 0:
                        break
                else:
                    break
        for j in range(range_y):
            for i in reversed(range(1, range_x)):
                if dot_dict[(i, j)] <= 0:
                    fill_dict[(i, j)] = 1
                    if dot_dict[(i-1, j)] > 0:
                        break
                else:
                    break
        with open('obstacle_smooth_fill.txt', 'w') as f:
            #f.write('P2\n')
            #f.write('15\n')
            for i in range(range_x):
                for j in range(range_y):
                    if fill_dict[(i, j)] > 0:
                        f.write(str('3'))
                    else:
                        f.write(str('0'))
                    #print(dot_dict[(i, j('0')])
                f.write('\n')
            f.write(str(int(-min_x*10//1))+' '+str(int(-min_y*10//1))+'\n')
            f.write(str(range_x)+' '+str(range_y))

#/fill
        #print('done')

#========= test ============
# if __name__ == '__main__':
#     while True:
#         make_obstacle_map()
#========= test ============


rospy.init_node('UVbot_obstacle', anonymous=True)
while not rospy.is_shutdown():
    make_obstacle_map()
rospy.spin()



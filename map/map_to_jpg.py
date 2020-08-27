#!/usr/bin/env python

import numpy as np
import cv2

with open('combine_map.txt', 'r') as f:
    lines = [line.strip() for line in f.readlines()]
range_x = int(lines[-1].split()[0])
range_y = int(lines[-1].split()[1])
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

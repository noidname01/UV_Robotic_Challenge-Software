import pcl
import numpy as np


with open('1597122874193567.pcd', 'r') as f:
    lines = [line.strip().split() for line in f.readlines()]

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

with open('new.pcd', 'w') as f:
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


#p = pcl.load('~/.ros/1597066607890791.pcd')

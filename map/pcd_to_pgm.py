import pcl
'''
input: .pcd file, z=0
output: .pgm file*2
(1) test_pgm.pgm
(2) test_pgm_smooth.pgm
'''

thres = 5

with open('new.pcd', 'r') as f:
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

total_x = (max_x - min_x) * 10
total_y = (max_y - min_y) * 10
range_x = int(total_x//1) + 1
range_y = int(total_y//1) + 1
print(range_x, range_y)

dot_dict = dict()
for i in range(range_x):
    for j in range(range_y):
        dot_dict[(i, j)] = 0
for d in data:
    dot_dict[(int(d[0]*10//1),int(d[1]*10//1))] += 1


with open('test_pgm.pgm', 'w') as f:
    f.write('P2\n')
    f.write(str(range_x)+' '+str(range_y)+'\n')
    f.write('15\n')
    for j in range(range_y):
        for i in range(range_x):
            if dot_dict[(i, j)] > 0:
                f.write(str('30'))
            else:
                f.write(str('0'))
            #print(dot_dict[(i, j('0')])
            f.write(' ')
        f.write('\n')

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

with open('test_pgm_smooth.pgm', 'w') as f:
    f.write('P2\n')
    f.write(str(range_x)+' '+str(range_y)+'\n')
    f.write('15\n')
    for j in range(range_y):
        for i in range(range_x):
            if dot_dict[(i, j)] > 0:
                f.write(str('30'))
            else:
                f.write(str('0'))
            #print(dot_dict[(i, j('0')])
            f.write(' ')
        f.write('\n')

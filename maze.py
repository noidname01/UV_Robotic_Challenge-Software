from mpl_toolkits.mplot3d import Axes3D
import math
import pyrealsense2 as rs
import matplotlib.pyplot as plt

from utils.loc import get_location, get_absolute_location, degree_to_arc, polar_to_cartesian
import yaml

# Depth Field of View -- 86 * 57 * 94 (horizontal * vertical * diagonal)

pipeline = rs.pipeline()
pipeline.start()

#load parameters
#if __name__=='__main__':
with open('config/params.yaml') as F:
    params = yaml.load(F, Loader=yaml.FullLoader)
    size = params['robot']['size']                   #(float,float,float): robot's size given the unit of length = 2cm
    r_detect = params['robot']['d_detect']           
    #pass_size = params['robot']['pass_size']         #(float,float): the range [width,height] robot checks if any obstacle d_detect ahead
    angle = params['robot']['angle']
    point_split = params['robot']['point_split']
    height_check = params['robot']['height_check']

class maze:
    def __init__(self, length, width, height):
        self.maze = [[[False for k in range(height)] for j in range(width)] for i in range(length)]
        self.rangex = length
        self.rangey = width
        self.rangez = height
    
    def set_thing(self, loc):
        x, y, z = loc[0], loc[1], loc[2]
        a, b, c = x//2, y//2, z//2
        self.maze[x][y][z] = True

    def delete_thing(self, loc):
        x, y, z = loc[0], loc[1], loc[2]
        a, b, c = x//2, y//2, z//2
        self.maze[a][b][c] = False

    def set_wall(self, a, b, c):
        #set the wall ax+by+cz=1
        pass

    def draw_model(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')
        xs = list()
        ys = list()
        zs = list()
        for i in range(self.rangex):
            for j in range(self.rangey):
                for k in range(self.rangez):
                    if self.maze[i][j][k] == True:
                        xs.append(i)
                        ys.append(j)
                        zs.append(k)
        ax.scatter(xs, ys, zs)
        plt.show()
        


class robot:
    def __init__(self, mz, pos=(0, 0, 0), direc=0, size=[20.,20.,80.], r_detect=20., angle=36., point_split=7, height_check=90):
        '''
        mz: the room model
        pos: (double, double, double), the initial position of the robot
        direc: double (in degree), the initial direction of the robot
        '''
        self.mz = mz
        self.pos = pos
        self.direc = direc
        self.size=size
        self.r_detect=r_detect
        self.r2_detect = (self.r_detect+self.size[0]/2)/2
        self.angle=3angle
        self.point_split=point_split
        self.height_check=height_check
        self.split_angle = self.angle/((self.point_split-1)//2)
        

    def get_pixel_set_maze(self, pixel, distance, direc):
        '''
        pixel: (int, int), the location of this pixel in the picture
        distance: double, the distance from the camera to the thing
        direc: double (in degree), the direction of the robot
        '''
        loc = get_location(pixel, distance)
        abs_loc = get_absolute_location(self.pos, self.direc, loc)
        self.mz = set_thing(abs_loc)

    def get_frame_set_maze(self):
        depth = False
        while not depth:
            frame = pipeline.wait_for_frames()
            depth = frame.get_depth_frame()
        for y in range(480):
            for x in range(640):
                distance = depth.get_distance(x, y)
                get_pixel_set_maze((x, y), distance, self.direc)
    
    def left_navi(self):
        '''
        input:
        output:
            action: (string), 'move_forward' or 'turn_left'
        '''
        #get the loc of checkpoints
        loc_array = [[self.pos[0], self.pos[1]]for i in range(self.point_split)]

        #1.first half of checkpoints at distance r_detect (outer arc)
        x_comp, y_comp = polar_to_cartesian(self.r_detect,self.direc)
        loc_array[0][0] += round(x_comp)
        loc_array[0][1] += round(y_comp)
        for i in range(1,(self.point_split-1)//2+1):
            x_comp, y_comp = polar_to_cartesian(self.r_detect,self.direc+i*self.split_angle)
            loc_array[i][0] += round(x_comp)
            loc_array[i][1] += round(y_comp)
            x_comp, y_comp = polar_to_cartesian(self.r_detect,self.direc-i*self.split_angle)
            loc_array[i+(self.point_split-1)//2][0] += round(x_comp)
            loc_array[i+(self.point_split-1)//2][1] += round(y_comp)

        #2.second half of checkpoints at distance (r_detect+'width_of_robot')/2  (inner arc)
        x_comp, y_comp = polar_to_cartesian(self.r2_detect,self.direc)
        loc_array[self.point_split][0] += round(x_comp)
        loc_array[self.point_split][1] += round(y_comp)
        for i in range(1,(self.point_split-1)//2+1):
            x_comp, y_comp = polar_to_cartesian(self.r2_detect,self.direc+i*self.split_angle)
            loc_array[self.point_split+i][0] += round(x_comp)
            loc_array[self.point_split+i][1] += round(y_comp)
            x_comp, y_comp = polar_to_cartesian(self.r2_detect,self.direc-i*self.split_angle)
            loc_array[self.point_split+i+(self.point_split-1)//2][0] += round(x_comp)
            loc_array[self.point_split+i+(self.point_split-1)//2][1] += round(y_comp)

        #check if any obstacle at these checkpoints
        for i in range(self.point_split*2):
            for k in range(1,self.height_check):
                if self.maze[loc_array[i][0],loc_array[i][1],k]:
                    return 'turn_left'
        return 'move_forward'



def get_location(pixel, distance):
    '''
    input: 
        pixel: (int, int), the location of this pixel in the picture
        distance: double, the distance from the camera to the thing (detected by the camera)
    output:
        loc: (double, double, double), the location of this pixel, given the origin = (0, 0)
    '''
    x, y = pixel[0], pixel[1]
    if x > 320 and y > 240:
        x = x - 320
        y = y - 240
        theta = math.atan(x/(320*math.sin(degree_to_arc(43)/math.cos(degree_to_arc(43)))))
        phi = math.atan(y/(240*math.sin(degree_to_arc(28.5)/math.cos(degree_to_arc(28.5)))))
        r = math.sqrt(pow(distance, 2)/(pow(1/math.cos(theta), 2)+pow(math.sin(phi)/math.cos(phi), 2)))
        a = r * math.sin(theta) / math.cos(theta)
        b = r
        c = r * math.sin(phi) / math.cos(phi)
        return (a, b, c)
    elif x < 320 and y > 240:
        x = 320 - x
        y = y - 240
        theta = math.atan(x/(320*math.sin(degree_to_arc(43)/math.cos(degree_to_arc(43)))))
        phi = math.atan(y/(240*math.sin(degree_to_arc(28.5)/math.cos(degree_to_arc(28.5)))))
        r = math.sqrt(pow(distance, 2)/(pow(1/math.cos(theta), 2)+pow(math.sin(phi)/math.cos(phi), 2)))
        a = r * math.sin(theta) / math.cos(theta)
        b = r
        c = r * math.sin(phi) / math.cos(phi)
        return (-a, b, c)
    elif x < 320 and y < 240:
        x = 320 - x
        y = 240 - y
        theta = math.atan(x/(320*math.sin(degree_to_arc(43)/math.cos(degree_to_arc(43)))))
        phi = math.atan(y/(240*math.sin(degree_to_arc(28.5)/math.cos(degree_to_arc(28.5)))))
        r = math.sqrt(pow(distance, 2)/(pow(1/math.cos(theta), 2)+pow(math.sin(phi)/math.cos(phi), 2)))
        a = r * math.sin(theta) / math.cos(theta)
        b = r
        c = r * math.sin(phi) / math.cos(phi)
        return (-a, b, -c)
    elif x > 320 and y > 240:
        x = x - 320
        y = 240 - y
        theta = math.atan(x/(320*math.sin(degree_to_arc(43)/math.cos(degree_to_arc(43)))))
        phi = math.atan(y/(240*math.sin(degree_to_arc(28.5)/math.cos(degree_to_arc(28.5)))))
        r = math.sqrt(pow(distance, 2)/(pow(1/math.cos(theta), 2)+pow(math.sin(phi)/math.cos(phi), 2)))
        a = r * math.sin(theta) / math.cos(theta)
        b = r
        c = r * math.sin(phi) / math.cos(phi)
        return (a, b, -c)


def get_absolute_location(pos, direc, loc):
    '''
    input:
        pos: (double, double, double), the location of the robot
        direc: double, the direction of the camera (or the direction of the robot) in degree
        loc: (double, double, double), the location of this pixel, given the origin = (0, 0)
    output:
        abs_loc: (double, double, double), the location of this pixel, given the origin = pos
    '''    
    arc = degree_to_arc(direc)
    x = loc[0]
    y = loc[1]
    z = loc[2]
    a = x * math.cos(arc) - y * math.sin(arc)
    b = x * math.sin(arc) + y * math.cos(arc)
    c = z
    return (a+pos[1], b+pos[2], c+pos[3])


def degree_to_arc(degree):
    return degree*2*math.pi/360


if __name__ == "__main__":
    mz = maze(10, 10, 10)
    mz.set_thing((1, 6, 8))
    mz.draw_model()

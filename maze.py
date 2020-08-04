import math
import pyrealsense2 as rs

# Depth Field of View -- 86 * 57 * 94 (horizontal * vertical * diagonal)

pipeline = rs.pipeline()
pipeline.start()

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


class robot:
    def __init__(self, mz, pos=(0, 0, 0), direc=0):
        '''
        mz: the room model
        pos: (double, double, double), the initial position of the robot
        direc: double (in degree), the initial direction of the robot
        '''
        self.mz = mz
        self.pos = pos
        self.direc = direc

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
    print(get_location((320, 240), 10))


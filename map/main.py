#!usr/bin/env python
import math
import cv2
import pyrealsense2 as rs
import serial
from utils.loc import get_location, get_absolute_location, degree_to_arc, polar_to_cartesian
import yaml
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

# Depth Field of View -- 86 * 57 * 94 (horizontal * vertical * diagonal)

def main_node():
    rospy.init_node('UVbot_main', anonymous=True)
    while not rospy.is_shutdown():
        main()

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

class robot:
    def __init__(self, mz, pos=(0, 0), direc=-90, a_direc = 0, size=[20.,20.,80.], r_detect=20., angle=36., point_split=7, height_check=90, safe_dist=5, vision_angle=None):
        '''
        mz: the room model
        pos: (int, int), the initial position of the robot
        direc: double (in degree), the initial direction of the robot  (for left_navi)
        a_direc: double (in degree), the initial direction of the robot (for A*)
        '''
        self.mz = mz
        self.astar_mz = mz
        self.safe_mz = mz
        self.pos = pos
        self.direc = direc
        self.a_direc = a_direc
        self.size=size
        self.r_detect=r_detect
        self.r2_detect = (self.r_detect+self.size[0]/2)/2
        self.angle=angle
        self.point_split=point_split
        self.height_check=height_check
        self.split_angle = self.angle/((self.point_split-1)//2)
        self.safe_dist = safe_dist
        self.vision_angle = vision_angle if vision_angle else {'hor':86.,'ver':57.,'dia':94.}                          #type:dict
        assert type(vision_angle)=='dict', 'the type of vision_angle should be dict'
        # open and set Serial
        self.ser=serial.Serial(“/dev/ttyUSB0”,9600,timeout=None)
        self.ser.open()

    def get_pixel_set_maze(self, pixel, distance, direc):
        '''
        pixel: (int, int), the location of this pixel in the picture
        distance: double, the distance from the camera to the thing
        direc: double (in degree), the direction of the robot (for left_navi)
        a_direc: double (in degree), the direction of the robot (for astar)
        '''
        loc = get_location(pixel, distance)
        abs_loc = get_absolute_location(self.pos, self.direc, loc)
        self.mz.set_thing(abs_loc)

    def get_frame_set_maze(self):
        depth = False
        while not depth:
            frame = pipeline.wait_for_frames()
            depth = frame.get_depth_frame()
        for y in range(480):
            for x in range(640):
                distance = depth.get_distance(x, y)
                self.get_pixel_set_maze((x, y), distance, self.direc)
    
    def left_navi(self):

        while(not self.is_trapped()): # need to fix to follow the map
            # update self.mz, self.astar_maze and self.loc
            self.combine_map_reader()
            self.update_loc()
            # step1. go straight until the following check return true
            self.go_straight()

            # check1. check if going to crush
            if self.crush_check():
                # turn right 90 degree
                self.turn_right()

                # turn left to find the way to move forward
                self.turn_to_find_path("left")
                
            # check2. check if it's time to turn left
            if self.turn_check():

                inch = self.get_specific_distance() // 10 
                rest = self.get_specific_distance() % 10
                
                # go straight a little bit
                self.go_straight(rest)

                for i in range(inch):
                    self.go_straight()

                    # set a variable to let inner loop can pass outward
                    shouldCrush = False
                    # check will crush or not
                    if self.crush_check():
                        
                        shouldCrush = True

                        # turn right 90 degree
                        self.turn_right()

                        # turn left to find the way to move forward
                        self.turn_to_find_path("left")

                        #jump out the loop
                        break
                
                
                if shouldCrush:
                    #  jump out of loop
                    continue

                # turn left 90 degree
                self.turn_left()

                # turn right and find the way to move forward
                self.turn_to_find_path("right")
            
    def navi(self, path):
        """ Give instruction to robot to go along path """
        for i in range(1,len(path)):
            dire = self.a_direc %360
            r, theta = cv2.cartToPolar(path[i][0] - path[i-1][0], path[i][1] - path[i-1][1] , angleInDegrees=True)
            r, theta = int(r[0]), int(theta[0])
            d = theta - dire  # -359 ~ 359
            if d = 0:
                self.go_straight(0.02*r)
            elif 0 < d <= 180:
                self.turn_left(d)
            elif 180 < d <360:
                self.turn_right(360 - d)
            elif -180 <= d < 0:
                self.turn_right(-d)
            elif -360 < d < -180:
                self.turn_left(360 - d)


    def go_straight(self, dist = 0.1):
        '''
        input:
            dist: distance, the distance you want the robot to move forward, it will keep move continuously in default
        output:
            none
        '''
        self.ser.write('f ')
        self.ser.write(dist)
        self.ser.write('\n')
        sleep(0.1)
        self.ser.read_until('c')

    def turn_left(self, deg = 90):
        '''
        input:
            deg: degree, the degree you want the robot to turn left, default would be 90 degree
        output:
            none
        '''
        self.ser.write('l ')
        self.ser.write(deg)
        self.ser.write('\n')
        sleep(0.1)
        self.ser.read_until('c')
        self.direc += deg
        self.a_direc += deg

    def turn_right(self, deg = 90):
        '''
        input:
            deg: degree, the degree you want the robot to turn right, default would be 90 degree
        output:
            none
        '''
        self.ser.write('r ')
        self.ser.write(deg)
        self.ser.write('\n')
        sleep(0.1)
        self.ser.read_until('c')
        self.direc -= deg
        self.a_direc -= deg

    def infinite_turn(self, direction):
        '''
        input:
            direction: string "left" or "right"
        output:
            none
        '''
        if direction == 'right':
            self.ser.write('r ')
            self.ser.write('i')
            self.ser.write('\n')
            while self.ser.in_waiting == 0:
                angle = int(self.ser.read().decode('utf-8').rstrip)
            self.direc -= angle
            self.a_direc -= angle
         elif direction == 'left':
            self.ser.write('l ')
            self.ser.write('i')
            self.ser.write('\n')
            while self.ser.in_waiting == 0:
                angle = int(self.ser.read().decode('utf-8').rstrip)
            self.direc += angle
            self.a_direc += angle

    def halt(self):
        '''
        HALT.
        '''
        self.ser.write('h')
        self.ser.write('\n')
        sleep(0.1)
        self.ser.read_until('c')
                
    def crush_check(self):
        '''
        description:
            crush_check is to check whether to get the crush on something. It should use the depth camera and check the frame it sends,

            if there are points or area in ""specific"" distance, it will return true, which means it will crush if we don't stop it.

            the specific distance will be the robot-safe-dist, which is a safety distance for the robot as the cushion, prevents it from the 

            affection of deviation 
        '''

        '''
        input:
        output:
            shouldStop: bool, means that robot should stop move forward
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
            if self.mz[loc_array[i][0],][loc_array[i][1]]:
                return True
        return False

    def turn_check(self):
        '''
        description:
            turn_check is to check whether is the time to turn, prevents it from missing the right turning time.

            It will track the ""specific"" point in the frame of depth camera, uses it to determine when to turn.

            the specific point is the translation of the position of camera/robot, I think the coordinate of the 'specific point'should be 

            ( self.pos.x - ( robot-width / 2 + robot-safe-dist ) , self.pos.y + ( robot-width / 2 + robot-safe-dist ) * tan(90 - robot-vision-angle / 2) ) 
            
            in cartesian coordinate system

            when it found this point becoming empty, it will return true, which means that it can turn "" left ""
 
        '''
        x,y = self.specific_point_coord('left')

        return not self.mz[x][y]
            
    def turn_to_find_path(self, direction):
        '''
        input:
            direction: left or right, which will be applied to different check.
        
        description:
            turn_to_find_path is to turn left or right to find the ""specific point"" of camera frame
            when it found this point becoming empty, it will return true, which means that it can "" go forward ""

        '''
        if direction == 'right':
            sp = self.specific_point_coord('right')
            if self.mz[sp[0]][sp[1]]:
                self.infinite_turn('right')
            while  self.mz[sp[0]][sp[1]]:
                sp = self.specific_point_coord('right')
            self.halt()
            return True
        elif direction == 'left':
            sp = self.specific_point_coord('left')
            if self.mz[sp[0]][sp[1]]:
                self.infinite_turn('left')
            while  self.mz[sp[0]][sp[1]]:
                sp = self.specific_point_coord('left')
            self.halt()
            return True
         
    def specific_point_coord(self, direction):
        '''
        input:
            direction: str, 'left' or 'right'
        output:
            coord: tuple, (x,y)
        description:
            it will return a tuple that indicate the coordintate of specific point
        '''
        assert direction in ['left','right'], 'direction(type:str) should be left or right'
        if direction=='left':
            return (round(self.pos[0]-(self.size[0]/2+self.safe_dist))\
            ,round(self.pos[1]+(self.size[1]/2+self.safe_dist)*math.tan(90-self.vision_angle['hor']/2)))
        elif direction=='right':
            return (round(self.pos[0]+(self.size[0]/2+self.safe_dist))\
            ,round(self.pos[1]+(self.size[1]/2+self.safe_dist)*math.tan(90-self.vision_angle['hor']/2)))

    def specific_point_depth(self):
        '''
        input:
            none
        output:
            depth: float, the depth of the point
            
        '''
        pass
    
    def get_specific_distance(self):
        '''
        input:
            none
        output:
            distance: float, the distance that the robot would move forward before turning
        '''
        return math.tan(self.vision_angle['hor']/2)*(self.safe_dist+0.5*(self.size[0]))

    def find_unknown_area(self):
        '''
        input:
            none
        output:
            (int,int), the coor of the nearest point located in the unknown area
            It'll return 'all done!' if the whole room is checked
        '''
        for d in range(1,self.mz.rangex+self.mz.rangey-1):   #d: distance(L1 norm) from robot
            for x in range(-d-1,d+1):              #x+y=d
                y = d-abs(x)
                if 0<=self.pos[0]+x<=self.mz.rangex and 0<=self.pos[1]+y<=self.mz.rangey and not self.safe_mz[self.pos[0]+x][self.pos[1]+y]:
                    return self.pos[0]+x, self.pos[1]+y
                elif 0<=self.pos[0]-x<=self.mz.rangex and 0<=self.pos[1]-y<=self.mz.rangey and not self.safe_mz[self.pos[0]-x][self.pos[1]-y]:
                    return self.pos[0]-x, self.pos[1]-y
        return 'all done!'
   
    def update_loc(self):
        with open('combine_map.txt', 'r') as f:
            lines = [line.strip().split() for line in f.readlines()]
            self.pos = (int(lines[-1].split()[0]), int(lines[-1].split()[1]))

    def combine_map_reader(self):
        '''
        read combine_map.txt, modify self.mz
        self.mz: 0 for road, 1 for disinfected region, 2 for path, 3 for wall, 4 for road around wall
        self.astar_mz: 0 for wall, 1 for road
        '''
        with open('combine_map.txt', 'r') as f:
            lines = [line.strip().split() for line in f.readlines()]
        mz = [[] for i in range(len(lines)-3)]
        self.rangex = len(lines)-3
        self.rangey = len(lines[0])
        for i in range(len(lines)-3):
            for j in range(len(lines[0])):
                mz[i].append(lines[i][j])
        self.mz = mz
        for i in range(len(lines)-3):
            for j in range(len(lines[0])):
                for a in range(-5,5):
                    for b in range(-5,5):
                        try:
                            if mz[i+a][j+b] == 3:
                                mz[i][j] = 4
                        except:
                            pass
        self.safe_mz = mz
        for i in range(len(lines)-3):
            for j in range(len(lines[0])):
                mz[i][j] = 0
                for a in range(-5,5):
                    for b in range(-5,5):
                        try:
                            if mz[i+a][j+b] == 3:
                                mz[i][j] = 1
                        except:
                            pass
        self.astar_mz = mz
        
        
        
    def is_trapped(self):
        #return bool
        #determine whether the robot is trapped (when left_navi)
        for i in range(self.pos[0]-1, self.pos[0]+2):
            for j in range(self.pos[1]-1, self.pos[1]+2)
                try:
                    if mz[i][j] == 0 and i != self.pos[0] and j != self.pos[1]:
                        return False
                except:
                    pass
        return True
    
    def is_clear(self):
        #Check if the robot has cleaned all the room
        #return bool
        for i in range(len(self.mz)):
            for j in range(len(self.mz[0])):
                if self.mz[i][j] == 0:
                    return False
        return True


class point:
    def __init__(self,x = 0,y = 0, name = "path"):
        self.x = x
        self.y = y
        self.weight = 0
        self.path_weight = 0
        self.have_search = False
        self.name = name
        self.part_path = [] 

    def get_weight(self):
        return self.weight

    def set_weight(self, origin):
        if isinstance(origin,point):
            self.weight = round(((self.x-origin.x)**2+(self.y-origin.y)**2)**(0.5))
        else:
            self.weight = origin

    def set_path_weight(self,weight):
        self.path_weight = weight

    def get_path_weight(self):
        return self.path_weight

    def get_have_search(self):
        return self.have_search

    def set_have_search(self,boo):
        self.have_search = boo

    def set_part_path(self,part_path):
        self.part_path = part_path

    def get_part_path(self):
        return self.part_path

class maze:
    def __init__(self,maze_list):
        self.maze = []
        self.maze_list = maze_list

    def create_maze(self):
        # choose left top as origin

        for y in range(len(self.maze_list)):
            newlist = []
            for x in range(len(self.maze_list[y])):
                if(self.maze_list[y][x]):
                    newlist.append(point(x = x,y = y))
                else:
                    newlist.append(point(name = "wall"))
            self.maze.append(newlist)

def print_point_list(l):
    for i in l:
        print(i.x,i.y)

def search_neighbor(weight_maze, current_x, current_y):
    try:
        if (current_y - 1) >= 0: 
            top_neighbor = weight_maze[current_y - 1][current_x]
        else:
            top_neighbor = None
    except:
        top_neighbor = None
    
    try:
        bottom_neighbor = weight_maze[current_y + 1][current_x]
    except:
        bottom_neighbor = None

    try:
        if (current_x - 1) >= 0: 
            left_neighbor = weight_maze[current_y][current_x - 1]
        else:
            left_neighbor = None
    except:
        left_neighbor = None

    try:
        right_neighbor = weight_maze[current_y][current_x + 1]
    except:
        right_neighbor = None


    neighbors = [top_neighbor, left_neighbor, bottom_neighbor, right_neighbor]

    # the next currentposition's candidate
    next_candidate = []

    for neighbor in neighbors:

        # if this neighbor exists
        if neighbor and neighbor.name != "wall":

            if not neighbor.get_have_search():

                
                # set neighbor's path weight = currentpoint's path weight + 1
                neighbor.set_path_weight(weight_maze[current_y][current_x].get_path_weight() + 1 )
                # set this neighbor has been searched
                neighbor.set_have_search(True)
                # add into part path
                
                new_part_path = weight_maze[current_y][current_x].get_part_path()+[]
                new_part_path.append(neighbor)
                neighbor.set_part_path(new_part_path)
                

                if neighbor.name == "G":
                    # if find Goal, return True
                    return True,[]

                else:
                    # this neighbor isn't the goal, keep find and let it join the candidate
                    next_candidate.append(neighbor)

            
            else:
                # this neighbor has been searched, we skip it
                continue

        else:
            # this neigbor doesn't exist, so skip
            continue

    return False, next_candidate

def astar(maze,start,goal):
    """
    input:
        start:
            type:           point
            description:    start point of astar
        goal:
            type:           point
            description:    end point of astar

    output:
        the list of points that the shortest path will go through
    """
    #=============== params ===========#
    start_x = start.x
    start_y = start.y
    goal_x = goal.x
    goal_y = goal.y

    current_x = start_x
    current_y = start_y

    neighbor_queue = []
    
    # the target path
    path = []
    #=============== params ===========#

    #===============build weight_maze ===========#

    for row in maze:
        newlist = []
        for column in row:
            if(column):
                column.set_weight(goal)
            else:
                column.set_weight(0)

    weight_maze = maze

    #===============build weight_maze ===========#

    #================start search ===============#

    # 1. set start/goal point:
    weight_maze[start_y][start_x].name = "S"
    weight_maze[start_y][start_x].set_have_search(True)
    
    new_part_path = weight_maze[start_y][start_x].get_part_path()
    new_part_path.append(weight_maze[start_y][start_x])
    weight_maze[start_y][start_x].set_part_path(new_part_path)
    
    weight_maze[goal_y][goal_x].name = "G"

    # 2. while loop until found G
    isG = False
    while(not isG):
        isG, nextCandidates = search_neighbor(weight_maze, current_x, current_y)

        # add new candidate to old ones
        neighbor_queue += nextCandidates
        

        # sort with path weight + weight, the least at first
        neighbor_queue = sorted(neighbor_queue, key = lambda neighbor:neighbor.get_path_weight() + neighbor.get_weight())
        
        # get whose total weight is the least and let it be the next position
        next_position = neighbor_queue.pop(0)
    
        current_x = next_position.x
        current_y = next_position.y

        # print(current_x,current_y)

    path = weight_maze[goal_y][goal_x].get_part_path()
    # print("x",current_x,"y",current_y)
    return path

def reduce_path(lst):
    """ Delete redundant points through the straight line, 
            only remains endpoints of the segment.

            input:
                    lst: (list of point) list of points
            output:
                    new_lst: (list of point) modified point list.
    """
    new_lst = [lst[0]]
    for i in range(1, len(lst)-1):
        if (lst[i+1].x - lst[i-1].x) * (lst[i+1].y - lst[i-1].y):
            new_lst.append(lst[i])
    new_lst.append(lst[-1])
    return new_lst

def main():
    mz = [[False for j in range(150)] for i in range(150)]
    bot = robot(mz)
    bot.combine_map_reader()
    bot.left_navi()
    while(not bot.is_clear):
        # find the path to next unknown area
        goal = bot.find_unknown_area()
        bot.update_loc()
        present = (bot.pos[0], bot.pos[1])
        a = maze(bot.astar_mz)
        a.create_maze()
        a = a.maze
        path = astar(m, point(present[0], present[1]), point(goal[0], goal[1]))
        path = reduce_path(path)
        path_lst = []
        for pt in path:
            path_lst.append((pt.x, pt.y))

        bot.navi(path_lst)      
        bot.left_navi()


if __name__ == "__main__":
    main_node()

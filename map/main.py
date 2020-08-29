#!usr/bin/env python
import math
import cv2
import serial
from utils.loc import get_location, get_absolute_location, degree_to_arc, polar_to_cartesian
import yaml
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

# Load parameters
with open('config/params.yaml') as F:
    params = yaml.load(F, Loader=yaml.FullLoader)
    size = params['robot']['size']        #(float,float,float): robot's size given the unit of length = 2cm
    r_detect = params['robot']['r_detect']        #(float):  threshold distance of checking obstacle (unit length: 2cm)
    angle = params['robot']['angle']         #(float): the angle range of obstacle detection
    point_split = params['robot']['point_split']    #(int): num of checkpoint for obstacle detecting
    height_check = params['robot']['height_check']      #(int): min height the robot can pass
    safe_dist = params['robot']['safe_dist']        #(int): min safe distance

class robot:
    '''
    description: 
        The robot object that handles all behaviors about the robot, including finding path, checking the map and delivering commands to arduino.

    args:
        mz: 2D array, the room model marking the obstacles, odometry and the disinfected area.
        pos: (int, int), the initial coordinates of the robot (according to the map).
        direc: float (in degree), the initial direction of the robot  (for left_navi system), degree increase when the robot turn left.
        a_direc: float (in degree), the initial direction of the robot (for A* system), degree increase when the robot turn left.
        size: (float, float, float), the size of the robot.
        r_detect: float, the distance the camera detection can reach.
        angle: float, the horizontal view angle of the camera.
        point_split: int, the number of the sample points when detecting if there are obstacles around the robot.
        safe_dist: int, the safe distance between the robot and the obstacles.
        vision_angle: dict, the three dimensions view angle of the camera. 
                      None, default.
        replan: bool, encounter tof obstacle and haven't replan

    attribute:
        mz, pos, direc, a_direc, size, r_detect, angle, point_split, safe_dist, vision_angle, replan
        astar_mz: 2D array, the room model marking the obstacles.
        safe_mz: 2D array, the room model marking the obstacles and the region around the obstacles.
        r2_detect: float, the distance between the place the camera detection can reach and the center of the robot.
        split_angle: float, the angle of each of the sample points when detecting if there are obstacles around the robot.
    '''
    def __init__(self, mz, pos=(0, 0), direc=-90, a_direc = 0, size=[20.,20.,80.], r_detect=20., angle=43., point_split=7, safe_dist=5, vision_angle=None, replan = False):
        
        self.mz = mz  # comprised of 0, 1, 2, 3, 4
        self.astar_mz = mz  # comprised of 0 and 1
        self.safe_mz = mz   # consider safety distance (obstacle thicken)
        self.pos = pos    #(int, int)
        self.direc = direc  # for left_navi
        self.a_direc = a_direc # for A*
        self.size=size  # (float, float, float)
        self.r_detect=r_detect
        self.r2_detect = (self.r_detect+self.size[0]/2)/2
        self.angle=angle
        self.point_split=point_split
        self.split_angle = self.angle/((self.point_split-1)//2)
        self.safe_dist = safe_dist
        self.vision_angle = vision_angle if vision_angle else {'hor':86.,'ver':57.,'dia':94.}       #type:dict
        assert type(vision_angle)=='dict', 'the type of vision_angle should be dict'
        self.replan = replan
        # open and set Serial
        self.ser=serial.Serial("/dev/ttyUSB0",9600,timeout=None)
        self.ser.open()

# serial command
    def SerialWrite(self,output):
        send = str(output).encode("utf-8")
        self.ser.write(send)
    
    def SerialReadLine(self):
        if self.ser.inWaiting():
            rv = self.ser.readline().decode("utf-8") 
            return rv
        return ""

# emergency check
    def map_correct(self, x, y, direc, obs):
        """
        description:
            receive obstacle error from tof, generate a txt file to pass the information to obstacle.py.
        input:
            x, y: (int, int) coordinate of robot when encountering tof obstacle
            direc: (float) a_direc of robot when encountering tof obstacle
            obs:  (int) 0, 1 or 2. (0: only left, 1: only right, 2: both)
        output:
            None
        """
        pt_list = []
        theta = direc*math.pi/180
        left = (x + round(7.3*math.cos(theta)-9.75*math.sin(theta)), \
                y + round(7.3*math.sin(theta)+9.75*math.cos(theta)) )
        right =(x + round(7.3*math.cos(theta)+9.75*math.sin(theta)), \
                y + round(7.3*math.sin(theta)-9.75*math.cos(theta)) )
        if obs == 0: # left only
            pt_list.append(left)
        elif obs == 1: # right only
            pt_list.append(right)
        elif obs == 2:
            pt_list.append(left)
            pt_list.append(right)
        with open('tof.txt', 'w') as f:
            f.write('\n'.join('%s %s' % x for x in pt_list))

    def emergency_check(self):
        """
        description:
            generally check if receives emergency msg (el, er, human)
        input:
            None
        output:
            msg: (str) if no emergency, return self.SerialReadLine()
            None: if encounter human and then human left
            True: if receive "tof error" ('el' or 'er') MUST PLAN AGAIN !
        """
        msg = self.SerialReadLine()
        obs = None # obstacle position. (0: only left, 1: only right, 2: both)
        if msg not in ['human', 'el', 'er']:
            return msg

        if msg == 'human':
            sleep(0.2)
            while not self.SerialReadLine() == 'resume':
                pass
            return None

        elif msg == 'el':
            obs = 0
        elif msg == 'er':
            obs = 1
        # position stamp
        pos_x, pos_y = self.pos
        direc = self.a_direc
        # wait for safety
        while not self.SerialReadLine() == 'c':
            if self.SerialReadLine() == 'er':
                obs = 2
        self.map_correct()
        return True

# basic motion command
    def go_straight(self, dist = 0.1):
        '''
        description: 
            To command the robot to go straight.
        input:
            dist: float (in meter), the distance you want the robot to move forward, it will keep move continuously in default
        output:
            None
        '''
        self.emergency_check() # check human
        self.SerialWrite('f ')
        self.SerialWrite(dist)
        self.SerialWrite('\n')
        while not self.emergency_check() == 'c':
            if self.emergency_check() == True: # encounter tof obstacle
                self.replan = True
                sleep(0.25)
                self.combine_map_reader()
                self.update_loc()
                return None

    def turn_left(self, deg = 90):
        '''
        description:
            To command the robot to turn left.
        input:
            deg: float (in degree), the angle you want the robot to turn left, default = 90 degree
        output:
            None
        '''
        self.emergency_check() # check human
        self.SerialWrite('l ')
        self.SerialWrite(deg)
        self.SerialWrite('\n')
        while not self.emergency_check() == 'c':
            if self.emergency_check() == True: # encounter tof obstacle
                self.replan = True
                sleep(0.25)
                self.combine_map_reader()
                self.update_loc()
                return None
        self.direc += deg
        self.a_direc += deg
        
    def turn_right(self, deg = 90):
        '''
        description: 
            To command the robot to turn right.
        input:
            deg: float (in degree), the angle you want the robot to turn right, default = 90 degree
        output:
            None
        '''
        self.emergency_check() # check human
        self.SerialWrite('r ')
        self.SerialWrite(dist)
        self.SerialWrite('\n')
        while not self.emergency_check() == 'c':
            if self.emergency_check() == True: # encounter tof obstacle
                self.replan = True
                sleep(0.25)
                self.combine_map_reader()
                self.update_loc()
                return None
        self.direc -= deg
        self.a_direc -= deg

# checking functions for left_navi
    def crush_check(self):
        '''
        description: 
            crush_check is to used to check if the robot is about to crush on something.
            According to the map and some calculation, return True if there are obstacle too nearby (safety distance)
        input:
            None
        output:
            shouldStop: bool, whether the robot should stop moving forward.
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
            if self.mz[loc_array[i][0]][loc_array[i][1]]:
                return True
        return False

    def turn_check(self):
        '''
        description:
            turn_check is used to check whether is the time to turn, preventing it from missing the right turning time.
            It will track the "specific point" to determine when to turn.
            The specific point is the translation of the position of camera/robot, whose coordinate of the "specific point" should be 
            ( self.pos.x - ( robot-width / 2 + robot-safe-dist ) , self.pos.y + ( robot-width / 2 + robot-safe-dist ) * tan(90 - robot-vision-angle / 2) ) 
            in cartesian coordinate system
        input:
            None
        output:
            bool, True if robot could turn left  (the specific point is empty)
        '''
        x,y = self.specific_point_coord('left')
        return not self.mz[x][y]

        
    def infinite_turn(self, direction):
        '''
        description: 
            Infinitely rotate (left or right), remember to halt !!!!
        input:
            direction: string "left" or "right"
        output:
            None
        '''
        assert direction in ['left','right'], 'direction(type:str) should be left or right'
        if direction == 'right':
            self.SerialWrite('r 0\n')
        elif direction == 'left':
            self.SerialWrite('l 0\n')

    def halt(self, direc = 0):
        '''
        description: 
            HALT the robot.
        input: 
            direc: 0, 1 or -1. 
            (0: forward before halt.    1: turning left before halt.    -1: turning right before halt)
        output:
            None
        '''
        self.SerialWrite('h\n')
        sleep(0.1)
        while not self.ser.inWaiting():
            pass
        msg = self.SerialReadLine()
        if msg == 'c':
            angle = 0
        else:
            angle = int(msg)
        self.direc += angle*direc
        self.a_direc += angle*direc


    def turn_to_find_path(self, direction):
        '''
        description:
            turn_to_find_path is to turn left or right to find the ""specific point"" of camera frame
            when it found this point becoming empty, it will return true, which means that it can "" go forward ""
        input:
            direction: String, 'left' or 'right', which will be applied to different check.
        output:
            None        
        '''
        assert direction in ['left','right'], 'direction(type:str) should be left or right'
        if direction == 'right':
            sp = self.specific_point_coord('right')
            if self.mz[sp[0]][sp[1]]:
                self.infinite_turn('right')
            while  self.mz[sp[0]][sp[1]]:
                sp = self.specific_point_coord('right')
            self.halt(-1)
        elif direction == 'left':
            sp = self.specific_point_coord('left')
            if self.mz[sp[0]][sp[1]]:
                self.infinite_turn('left')
            while  self.mz[sp[0]][sp[1]]:
                sp = self.specific_point_coord('left')
            self.halt(1)
        return True

# specific value         
    def specific_point_coord(self, direction):
        '''
        description:
            it will return a tuple that indicate the coordintate of specific point
        input:
            direction: string, 'left' or 'right'
        output:
            coord: tuple (int, int), (x,y) of the specific point
        '''
        assert direction in ['left','right'], 'direction(type:str) should be left or right'
        if direction=='left':
            return (round(self.pos[0]-(self.size[0]/2+self.safe_dist))\
                ,round(self.pos[1]+(self.size[1]/2+self.safe_dist)*math.tan(90-self.vision_angle['hor']/2)))
        elif direction=='right':
            return (round(self.pos[0]+(self.size[0]/2+self.safe_dist))\
                ,round(self.pos[1]+(self.size[1]/2+self.safe_dist)*math.tan(90-self.vision_angle['hor']/2)))

    def get_specific_distance(self):
        '''
        description: 
            To get the distance that the robot have to move forward before turning.
        input:
            None
        output:
            distance: float, the distance that the robot would move forward before turning
        '''
        return math.tan(self.vision_angle['hor']/2)*(self.safe_dist+0.5*(self.size[0]))

# update function
    def update_loc(self):
        """
        description: 
            Update the present coordinate on the map. Update self.pos.
        input: 
            None
        output: 
            None
        """
        with open('combine_map.txt', 'r') as f:
            lines = [line.strip().split() for line in f.readlines()]
            self.pos = (int(lines[-1].split()[0]), int(lines[-1].split()[1]))

    def combine_map_reader(self):
        '''
        description: 
            Read combine_map.txt and update self.mz.
            self.mz: 0 for road, 1 for disinfected region, 2 for path, 3 for wall, 4 for road around wall. 
            self.astar_mz: 0 for wall, 1 for road. 
        input:
            None
        output:
            None
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

# state function              
    def is_trapped(self):
        """
        description:    
            To determine whether the robot is trapped (when doing left_navi())
        input:
            None
        output:
            bool, True if trapped.
        """
        for i in range(self.pos[0]-1, self.pos[0]+2):
            for j in range(self.pos[1]-1, self.pos[1]+2):
                try:
                    if mz[i][j] == 0 and i != self.pos[0] and j != self.pos[1]:
                        return False
                except:
                    pass
        return True
    
    def is_clear(self):
        """
        description:
            Check if the robot has cleaned all the room
        input:
            None
        output:
            Bool, true if all the room is clear.
        """
        for i in range(len(self.mz)):
            for j in range(len(self.mz[0])):
                if self.mz[i][j] == 0:
                    return False
        return True

# navigation 
    def left_navi(self):
        """
        description:
            This method includes the algorithm to lead the robot move beside the obstacles, and keep the robot being right to the obstacles. If there is any area beside the robot hasn't been disinfected, the robot will keep disinfecting. Otherwise, the robot will stop and the end the method.
        input:
            None
        output:  
            None
        """
        while(not self.is_trapped()): # need to fix to follow the map
            # update self.mz, self.astar_maze and self.loc
            self.combine_map_reader()
            self.update_loc()
            # step1. go straight until the following check return true
            self.go_straight()
            self.replan = False

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

    def find_unknown_area(self):
        '''
        description:
            Find if there is any area that hasn't been disinfected by the robot. If there is, select one the point that hasn't been disinfected and is the nearest to the robot, and return the coordinate of it.
        input:
            None
        output:
            Tuple (int,int), the coordinate of the nearest point located in the undisinfected area. (If there is area that hasn't been disinfected.)
            String, 'all done!', if the whole room is checked to be disinfected.
        '''
        for d in range(1,self.rangex+self.rangey-1):   #d: distance(L1 norm) from robot
            for x in range(-d-1,d+1):              #x+y=d
                y = d-abs(x)
                if 0<=self.pos[0]+x<=self.rangex and 0<=self.pos[1]+y<=self.rangey and not self.safe_mz[self.pos[0]+x][self.pos[1]+y]:
                    return self.pos[0]+x, self.pos[1]+y
                elif 0<=self.pos[0]-x<=self.rangex and 0<=self.pos[1]-y<=self.rangey and not self.safe_mz[self.pos[0]-x][self.pos[1]-y]:
                    return self.pos[0]-x, self.pos[1]-y
        return 'all done!'

    def find_route(self):
        """
        description:
            Find the next unknown area, then use A* algorithm to get the optimal path.
        input:
            None
        output:
            path_lst: (list) list of tuples, the optimal path solution
        """
        goal = self.find_unknown_area()
        self.update_loc()
        present = (self.pos[0], self.pos[1])
        a = maze(self.astar_mz)
        a.create_maze()
        a = a.maze
        path = astar(a, point(present[0], present[1]), point(goal[0], goal[1]))
        path = reduce_path(path)
        path_lst = []
        for pt in path:
            path_lst.append((pt.x, pt.y))
        return path_lst

    def navi(self, path):
        """
        description: 
            Give instruction to robot to go along the path.
        input: 
            path: an array, each element is a coordinate of the point that on the path from the location of the robot now to the destination.
        output:
            None
        """
        for i in range(1,len(path)):
            direc = self.a_direc %360
            r, theta = cv2.cartToPolar(path[i][0] - path[i-1][0], path[i][1] - path[i-1][1] , angleInDegrees=True)
            r, theta = int(r[0]), int(theta[0])
            d = theta - direc  # -359 ~ 359
            if d == 0:
                pass
            elif 0 < d <= 180:
                self.turn_left(d)
            elif 180 < d <360:
                self.turn_right(360 - d)
            elif -180 <= d < 0:
                self.turn_right(-d)
            elif -360 < d < -180:
                self.turn_left(360 - d)
            self.go_straight(0.02*r)
            
            if self.replan:
                path_lst = self.find_route()
                self.replan = False
                self.navi(path_lst)


# class and functions for A* algorithm      
class point:
    '''
    description:
        The type of the points on the map when doing the A* algorithm.
    
    arg:
        x: int, the x_coordinate of the point.
        y: int, the y_coordinate of the point.
        name: string, the name of the point.
    
    attribute:
        x, y, name, 
        weight: int, the weight of the point decided by the map.
        path_weight: int, the weight of the point decided by the distance to the origin.
        part_path: list, each element is a point on a distinct path.
    '''
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
    '''
    description:
        The room model used in A* algorithm.
        
    arg:
        maze_list: list, 2D array representing the map, each of the element is an integer.
        
    attribute: 
        maze_list,  
        maze: list, 2D array, each of the element is a point object.
    '''
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

def search_neighbor(weight_maze, current_x, current_y):
    '''
    description:
        The function that use in search neighboring area, 
        
    input:
        weight_maze: list, 2D array that points in map are all have been calculated its own weight
        current_x: int, current x coordinate
        current_y: int, current y coordinate
        
    output: 
        isFindG: bool, true if find G
        next_candidate, list, points that would be added to search queue
    '''
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
    description: 
        A* (pronounced "A-star") is a graph traversal and path search algorithm, 
        which is often used in many fields of computer science due to its completeness, 
        optimality, and optimal efficiency.
    input:
        start:
            type:           point
            description:    start point of astar
        goal:
            type:           point
            description:    end point of astar

    output:
        The list of points that the shortest path will go through.
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
    """ 
    description: 
        Delete redundant points through the straight line; only remains endpoints of the segment.
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

def print_point_list(l):
    for i in l:
        print(i.x,i.y)

# main
def main():
    '''
    description: 
        The main function of the robot. Contains stage1 (left_navi) and stage2 (A* algorithm and left_navi).
    input:
        None
    output:
        None
    '''
    mz = [[False for j in range(150)] for i in range(150)]
    bot = robot(mz)
    bot.combine_map_reader()
    bot.update_loc()
    bot.left_navi()
    while(not bot.is_clear):
        # find the path to next unknown area
        path_lst = bot.find_route()
        bot.navi(path_lst)      
        bot.left_navi()

def main_node():
    """
    description: 
        Create 'UVbot_main' node and start sterilize the entire room automatically. 
    input: 
        None
    output: 
        None
    """
    rospy.init_node('UVbot_main', anonymous=True)
    while not rospy.is_shutdown():
        main()

if __name__ == "__main__":
    main_node()

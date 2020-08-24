"""
1 stand for path
0 stand for wall
"""

test_maze = [[1,1,1,0,0,0,0],
             [1,0,1,0,0,0,0],
             [1,0,1,1,1,1,1],
             [1,0,1,0,0,0,1],
             [1,0,1,0,1,1,1],
             [1,0,1,0,1,0,0],
             [1,1,1,1,1,1,1]]


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

    def set_weight(self,origin):
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

        

if __name__ == "__main__":
    m = maze(test_maze)
    m.create_maze()
    m = m.maze
    path = astar(m, point(3,2), point(6,6))
    print_point_list(path)
    
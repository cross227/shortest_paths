from pyamaze import maze, agent, COLOR, textLabel
from queue import PriorityQueue

#A* takes into the goal cell while searching for the next cell (informed search)
#BFS takes the neighbors into account while searching (uninformed search)

#cost assoc. -> f(n) = g(n)+h(n) ...MIGHT NEED TO EXPAND LATER
#g(n) - cost of the path from 'start' node to node 'n' (not the goal just yet)
#h(n) - heuristic fcn that estimates cost of the cheapest path from n to the goal
####### Euclidean Dist vs Manhattan Dist

#NOTE: if we start and aim for a node with 2 moves then (assuming each edge = 1) then g(n) = 2
####### and then the from that new pt (not yet visited) the Euclidean (Resultant) dist is calculated
###### OR you can use the 'Manhattan" Distance which are the X,y components of the Resultant dist to reach the goal

### and so if the Manhattan dist (h(n) = 4 then the f(n) = 2+4 = 6 as the cost

#NOTE2: using a 4,4 maze ...at the start point (4,4) to goal (1,1)
####### g(n) = 0 b/c there has been no dp and h(n) =6 (via Manhattan dist)

#NOTE3: if 2 potential neighbors have the same f(n) but 1 has a 'lower h(n)' cost then THAT node will be visited

## will use "Priority Queue" to implement b/c the node value will come based on Priority and NOT just entry level


##1.0 Calculates the Manhattan dist btwn current agent cell pos and goal cell
### denotes coordinates as a tuple
def h(cell_1, cell_2):
    x1,y1 = cell_1
    x2,y2=cell_2
    return abs(x1-x2) +abs(y1-y2)


def aStar(m, start=None):
    if start is None:
        start = (m.rows, m.cols)
    # stores cells that are candidates for exploration based on priority
    # (det. by sum of remaining cost and chooses lower cost cell)
    open = PriorityQueue()
    # open.put(<h(n)_cost + start_cost>, h(n) cost, <cell name/value itself>)
    # here the "start" cell is added to "open"
    #priority tuple has 3 elements: total cost "f(n)", h(n) cost and "start cell coors"
    open.put((h(start, m._goal), h(start, m._goal), start))
    #apply f(n) = g(n) + h(n) - NOTE h(n) is above
    #initially g(n) and h(n) are 'inf so f(n) = inf
    aPath = {} #dict that stores the path

    #all g_score and f_score cells set to inf except the start cell for g(n) and f(n)
    g_score = {}
    for row in m.grid:
        g_score[row] = float("inf")
    #g(start) = 0
    g_score[start] = 0
    f_score = {row: float("inf") for row in m.grid}
    #f(start) set to heuristic value btw start and goal - Manhattan dist value
    f_score[start] = h(start, m._goal)
    searchPath = [start]

    # "while not open.empty()" loop continues as long as there are cells that are available for exploration in PriorityQueue
    while not open.empty():
        currCell = open.get()[2] #cell w/lowest f_score is removed from queue for processing and that cell returns a
                                #tuple via "open.get()" with [2] being the extracted cell coors
        searchPath.append(currCell) #those [2] cell coors are added to searchPath
        #always run a check to see if the "currCell" is the "goal" cell
        #if so the algorithm found a path to goal cell and the "while not open.empty()" loop is broken
        if currCell == m._goal:
            break
        # remember in the 'maze' ESNW already have established ASCII values
        for d in 'ESNW':
            if m.maze_map[currCell][d] == True: #checks if there's a valid path in dir 'd' from 'currCell' using maze_map
                if d == 'E':
                    childCell = (currCell[0], currCell[1] + 1)
                elif d == 'W':
                    childCell = (currCell[0], currCell[1] - 1)
                elif d == 'N':
                    childCell = (currCell[0] - 1, currCell[1])
                elif d == 'S':
                    childCell = (currCell[0] + 1, currCell[1])
                # and the idea is to have LOWER scores as you advance
                # and so once you enter a cell with a lower g(score) then ultimately the f_score will be lower
                # and these are established as the visited cell with its respective scores via "open.put()
                ## NOTE: path visited by ALGO and NOT by the vehicle yet so the scores are "temporary"
                temp_g_score = g_score[currCell] + 1 #this is the newly discovered path to childCell
                temp_f_score = temp_g_score + h(childCell, m._goal)

                # REMEMBER: if the scores of 2 cells are the same but one has a lower "h_score" then THAT cell is visited
                if temp_f_score < f_score[childCell]:
                    aPath[childCell] = currCell #stores the childCell that was ACTUALLY accepted by algo path
                    g_score[childCell] = temp_g_score #the accepted path is now ESTABLISHED as the g_score for travel cost
                    f_score[childCell] = g_score[childCell] + h(childCell, m._goal) #updated f_score via algo path
                    open.put((f_score[childCell], h(childCell, m._goal), childCell)) #updated location and restarts algo cycle/loop

    #ONCE the goal is reached by the Algorithm Path (not the vehicle) then the actual path traveled is stored for the vehicle
    #to travel from start to finish
    backTrack={}
    cell=m._goal
    while cell!=start:
        #the fwdPath will be "key" of the "aPath"
        # initially at THIS POINT the aPath[cell]= <goal cell> so it starts at goal and works backwards
        backTrack[aPath[cell]]=cell #sets each predecessor cell aPath[cell] as a key pointing to the "m._goal" cell
        cell=aPath[cell] #this builds the path from start to goal

    return searchPath, aPath, backTrack



myMaze=maze(33,15)
myMaze.CreateMaze(6,4,loopPercent=100) #loopPercent - is the maze complexity

searchPath,aPath,backTrack=aStar(myMaze,(1,12))

#'a' is the start point at (x1,y1) and path traces blue
a=agent(myMaze,1,12,footprints=True,color=COLOR.blue,filled=True)
#'b' is the goal point at (m._goal_x, m._goal_y) and traces yellow
b=agent(myMaze,6,4,footprints=True,color=COLOR.yellow,filled=True,goal=(1,12))
c=agent(myMaze,1,12,footprints=True,color=COLOR.red,goal=(6,4))
myMaze.tracePath({a:searchPath},delay=200)
myMaze.tracePath({b:aPath},delay=200)
myMaze.tracePath({c:backTrack},delay=200)

l=textLabel(myMaze,'A Star Path Length',len(backTrack)+1)
l=textLabel(myMaze,'A Star Search Length',len(searchPath))

myMaze.run()

# m=maze(10,10)
# m.CreateMaze()
# path=aStar(m)
# a=agent(m)
# m.tracePath({a:path})
# m.run()




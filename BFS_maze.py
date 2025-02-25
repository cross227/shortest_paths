#BFS_maze

from pyamaze import maze, agent, textLabel, COLOR
from collections import deque

#create a 5x5 maze as a small sample
# m=maze(5,5)
# m.CreateMaze()
## prints the maze at its start point and lists the possible directions
## HEREs an example output
### {(1, 1): {'E': 0, 'W': 0, 'N': 0, 'S': 1}, (2, 1): {'E': 0, 'W': 0, 'N': 1, 'S': 1}, and so on
#### NOTE: E,W,N,S are all predefined values from the ASCII and can be applied
# print(m.maze_map)
# # "m.run" gives a visual of the actual maze
# m.run()

def searchBFS(m, start=None):
    if start is None:
        #establishes the starting point at the VERY BOTTOM of the maze
        start =(m.rows, m.cols)
    frontier = deque() #double-ended queue
    # ## also to remove use "frontier.popleft()" and the FIFO kicks in
    ##### use "frontier.pop()" to apply LIFO
    frontier.append(start) #adds 'start' position (parent) to frontier and once the 'parent' visits another node (child)
    # then the visited value will become the parent and appended in frontier while the former parent is removed and
    # stroed in "explored" as a visited node
    bfsPath = {}
    explored = [start]
    bfsSearch =[]

    while len(frontier)>0:
        #extracts value from frontier and is assigned as "currCell"
        currCell=frontier.popleft()
        #so if the goal point is reached then BREAK the while loop and return the stored values in
        ### bfsPath, bSearch (note: might be more values to return seen later)
        if currCell==m._goal:
            break
        #remember in the 'maze' ESNW already have established ASCII values
        for _direction in 'ESNW':
            if m.maze_map[currCell][_direction]==True:
                if _direction=='E':
                    # currCell[0] = keys (coor pts) & currCell[1] = direction traveled
                    #ex) (1, 1): {'E': 0, 'W': 0, 'N': 0, 'S': 1}
                    childCell=(currCell[0],currCell[1]+1)
                elif _direction=='W':
                    childCell = (currCell[0], currCell[1] -1)
                elif _direction == 'S':
                    childCell = (currCell[0] + 1, currCell[1])
                elif _direction == 'N':
                    childCell = (currCell[0] - 1, currCell[1])
                if childCell in explored:
                    continue
                #now that a new cell has been visited it gets appended into frontier and stored as 'explored'
                frontier.append(childCell)
                explored.append(childCell)
                # sets the parent of the neighbor cell 'childCell' to current cell 'currCell' b/c the
                # neighbor (childCell) is NOW the parent
                bfsPath[childCell]=currCell
                bfsSearch.append(childCell)
            # print(f'{bfsPath}')
    #the fwdPath stores the Shortest Path values that are visited
    backTrack={}
    #set a "goal" location
    cell=m._goal
    #while cell != start
    while cell!=start:
        backTrack[bfsPath[cell]]=cell
        cell=bfsPath[cell]


    return bfsSearch, bfsPath, backTrack


#use this when the goal can be altered
if __name__=='__main__':
    m = maze(10,10)
    # 'm.CreateMaze(4,2,loopPercent=100)' has the "Goal" coors
    #def CreateMaze(self, goal_row, goal_col, loopPercent): -- these are the args for CreateMaze
    m.CreateMaze(4,2,loopPercent=100)
    bfsSearch,bfsPath,backTrack=searchBFS(m)
    # 'a' traverses thru maze and is tracked via 'm.tracePath({a:bSearch},delay=100)'
    a=agent(m, footprints=True, color=COLOR.green, shape='square')
    #'b' follows the shortest 'fwdPath
    b=agent(m, footprints=True, color=COLOR.yellow, shape='square', filled=False)
    # 'c' generates the shortest BFS path via 'm.tracePath({c: bfsPath}, delay=100)'
    c = agent(m,4,2, footprints=True, color=COLOR.cyan, shape='square', filled=True, goal=(m.rows,m.cols))
    m.tracePath({a:bfsSearch},delay=100)
    m.tracePath({c: bfsPath}, delay=100)
    m.tracePath({b: backTrack}, delay=100)
    m.run()


# def searchBFS(m):
#     start = (m.rows, m.cols)
#     frontier = [start]
#     explored = [start]
#     bfsPath={}
#     while len(frontier) > 0:
#         currCell = frontier.pop(0)
#         if currCell == m._goal:
#             break
#         for _dir in 'ESNW':
#             if m.maze_map[currCell][_dir] == True:
#                         if _dir=='E':
#                             # currCell[0] = keys (coor pts) & currCell[1] = direction traveled
#                             #ex) (1, 1): {'E': 0, 'W': 0, 'N': 0, 'S': 1}
#                             childCell=(currCell[0],currCell[1]+1)
#                         elif _dir=='W':
#                             childCell = (currCell[0], currCell[1] -1)
#                         elif _dir == 'S':
#                             childCell = (currCell[0] + 1, currCell[1])
#                         elif _dir == 'N':
#                             childCell = (currCell[0] - 1, currCell[1])
#                         if childCell in explored:
#                             continue
#                         frontier.append(childCell)
#                         explored.append(childCell)
#                         #bfsPath[childCell]=currCell sets the "reverse" path for tracking
#                         bfsPath[childCell]=currCell
#     #now we need to highlight and keep tabs of the forward path since the reverse is stored
#     fwdPath={}
#     #start at the goal cell until reach the start cell
#     # cell=(1,1)
#     cell=m._goal
#     while cell!=start:
#         #value of the fwdPath will be the KEY of the bfsPath
#         fwdPath[bfsPath[cell]]=cell
#         cell=bfsPath[cell]
#     return fwdPath

#USE this when the cell=(1,1) goal is HARD CODED
# if __name__=='__main__':
#     m = maze(10, 7)
#     m.CreateMaze(1,1,loopPercent=100)
#     path = searchBFS(m)
#     # create Agent and follow its 'footprints' along the path
#     a = agent(m, footprints=True)  # placed at start cell of maze
#     # 'd' in tracePath - method to trace path and so...
#     ## the agent 'a' follows along 'path' in searchBFS maze
#     m.tracePath({a: path})
#     m.run()











"""
Discussion of heuristic function selection:

Euclidean distance, which calaulated by sqrt((x1-x2)^2+ (y1-y2)^2), the length of a line segment between the two nodes.
when node move at any angle, euclidean distance is a good selection.

Manhattan distance, which calculated by |X1-X2|+|Y1-Y2|, the distance between two points measured along axes at right angles.
when node move at right angles, ie. moving at square grid, only x,y 4 directions, Manhattan distance is a good selection.

Diagonal distance heuristic, when the node can move diagonally, ie. moving at 8 directions, 4 directions at right angles and 4 directions at diagonal angle.
For Diagonal distance heuristic function, we have to calcuate diagonal part, saving non-diagonal steps, and
non-diagonal part, computing the number of steps which can not take by diagoanl moving.

dx = abs(x1 - x2)
dy = abs(y1 - y2)

Heuristic function = D*(dx + dy) + (D2 - 2 * D) * min(dx, dy)

D2 is the cost of moving diagonally
D is the cost of moving at right angle
If D = 1, D2 = 1, this is Chebyshev distance
If D= 1, D2 = sqrt(2),  this is the octile distance

reference: 
http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html

"""


import math
from queue import PriorityQueue


def shortest_path(Map, start, goal):
    print("shortest path called")
    # initial
    # use priority queue to save visited nodes for search
    nodeSearchQueue = PriorityQueue()    
    # set initial start node
    nodeSearchQueue.put((0, start))
    
    previousVisitNodes = {start: None}
    Score = dict()
    Score[start] = 0
    # search nodes
    while not nodeSearchQueue.empty():       
        # get min total score node from priority queue
        currentNode = nodeSearchQueue.get()[1]    
        # print((currentNode))
        # found the goal
        if currentNode == goal:
            break
        
        # search neigbor nodes
        for neighbor in Map.roads[currentNode]:            
            updateScore = Score[currentNode] + distanceEstimate(Map.intersections[currentNode], Map.intersections[neighbor])

            if neighbor not in Score or (updateScore < Score[neighbor]):
                Score[neighbor] = updateScore  

                nodeSearchQueue.put((updateScore, neighbor))
                previousVisitNodes[neighbor] = currentNode

        
    return constructPath(previousVisitNodes, start, goal)


def constructPath(previousVisitNodes, start, goal):
    # construct search path according to previousVisitNodes dictionary
    currentNode = goal
    path = [currentNode]
    while currentNode != start:
        currentNode = previousVisitNodes[currentNode]
        path.append(currentNode)            
    path.reverse()
    return path

def distanceEstimate(currentNode, targetNode):
    distance = math.sqrt((currentNode[0] - targetNode[0])**2 + (currentNode[1] - targetNode[1])**2 )    
    return distance   


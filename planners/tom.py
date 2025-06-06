import numpy as np
from typing import List, Tuple, Optional

class MimiNode:
    current = None
    pursued = None
    pursuer = None
    mover = -1
    depth = 0
    parent = None
    bestChild = None
    children = []
    value = 0
    def __init__(self, current=None, pursued=None, pursuer=None, depth=0, parent=None, mover=-1):
        self.current = current
        self.pursued = pursued
        self.pursuer = pursuer
        self.depth = depth
        self.parent = parent
        self.children = []
        self.value = 0
        self.mover = mover

def best_action(root: MimiNode):
        """
        Returns the best action from the root node of the minimini tree.
        """
        if not root.children:
            return None
        bestMove = None
        bestValue = float('inf')
        for child in root.children:
            if child.value < bestValue and tuple(child.current - root.current) != (0,0):
                bestValue = child.value
                bestMove = child.current - root.current

        #best_child = min(root.children, key=lambda x: x.value)
        return bestMove



def backValue(node: MimiNode):
    if node is None:
        return 0
    




def AnotherAnotherStar(s, grid, start, end, avoid):
    rows, cols = len(grid), len(grid[0])
    directions = [(0,1),(1,1),(1,0),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1), (0,0)]
    directions2 = [(0,1),(1,1),(1,0),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1)]
    
    pathWeight = {start: 0}
    parent = {start: None}
    

    open = [start]

    found = False
    while not found and len(open) > 0:
        node = None
        for newNode in open:
            if node is None or pathWeight[newNode] <= pathWeight[node]:
                node = newNode

        if node == end:
            found = True
            break

        nodeVal = {}
        open.remove(node)

        for dir in directions:
            newNode = (node[0] + dir[0], node[1] + dir[1])
            nodeVal[newNode] = hurst(newNode, end,avoid,rows,cols,grid) + dist(newNode, end)
            if newNode not in list(pathWeight.keys()):
                pathWeight[newNode] = pathWeight[node]
                #if 0 <= newNode[0] < rows and 0 <= newNode[1] < cols and grid[newNode[0]][newNode[1]] == 0:
                parent[newNode] = node
                open.append(newNode)
            #if newNode == end:
                #found = True                 
        
        
        for d in directions:
            nodeToBe = (node[0] + d[0], node[1] + d[1])
            
            for i in range(3):
                newNode = None
                if(i == 0):
                    newNode = (node[0] + -d[1], node[1] + d[0])
                elif(i == 1):
                    newNode = nodeToBe
                else:
                    newNode = (node[0] + d[1], node[1] + -d[0])
            
                pathWeight[nodeToBe] += nodeVal[newNode]*s.probabiliry[i]
             
             

    if found:
            path = []
            parentNode = end
            while parentNode is not None:
                path.append(parentNode)
                if parent[parentNode] is None:
                    break
                parentNode = parent[parentNode]
            return path[::-1]
    return None

def hurst(current, end, pursuer, rows, cols, grid):
    if tuple(current) == tuple(pursuer) or 0 > current[0] >= rows or 0 > current[1] >= cols or (0 <= current[0] < rows and 0 <= current[1] < cols and grid[current[0]][current[1]] == 0):
        return 9999999999999999
    temp = abs((current[0] - end[0])^2) + abs((current[1] - end[1])^2) **0.5
    temp2 = abs((current[0] - pursuer[0])^2) + abs((current[1] - pursuer[1])^2) **0.5

    if temp2 == 0:
        return 9999999999999999


    return ((1/temp2) *temp)


class PlannerAgent:
    directions2 = np.array([[0,1],[1,1],[1,0],[-1,1],[-1,0],[-1,-1],[0,-1],[1,-1]])
    probabiliry = np.array([.3,.3,.3])
    currentCount = 3
    directionsMoved = np.array([1,1,1])
    lastPos = np.array([0,0])
    lastDir = np.array([0,0])

    def updateProb(self, current: np.ndarray):
        self.currentCount += 1
        actualDir = current - self.lastPos
        if actualDir[0] == self.lastDir[0] and actualDir[1] == self.lastDir[1]:
            self.directionsMoved[1] += 1
        elif actualDir[0] == -self.lastDir[1] and actualDir[1] == self.lastDir[0]:
            self.directionsMoved[0] += 1
        elif actualDir[0] == self.lastDir[1] and actualDir[1] == -self.lastDir[0]:
            self.directionsMoved[2] += 1
        self.probabiliry[0] = self.directionsMoved[0] / self.currentCount
        self.probabiliry[1] = self.directionsMoved[1] / self.currentCount
        self.probabiliry[2] = self.directionsMoved[2] / self.currentCount
    
    def __init__(self):
        lastPos = np.array([0,0])
        pass


    
    def plan_action(self, world: np.ndarray, current: np.ndarray, pursued: np.ndarray, pursuer: np.ndarray) -> Optional[np.ndarray]:
        """
        Computes a action to take from the current position caputure the pursued while evading from the pursuer

        Parameters:
        - world (np.ndarray): A 2D numpy array representing the grid environment.
        - 0 represents a walkable cell.
        - 1 represents an obstacle.
        - current (np.ndarray): The (row, column) coordinates of the current position.
        - pursued (np.ndarray): The (row, column) coordinates of the agent to be pursued.
        - pursuer (np.ndarray): The (row, column) coordinates of the agent to evade from.

        Returns:
        - np.ndarray: one of the 9 actions from 
                              [0,0], [-1, 0], [1, 0], [0, -1], [0, 1],
                                [-1, -1], [-1, 1], [1, -1], [1, 1]
        """
        
        directions = np.array([[0,0], [-1, 0], [1, 0], [0, -1], [0, 1],
                                   [-1, -1], [-1, 1], [1, -1], [1, 1]]) 
        
        directions2 = np.array([[0,1],[1,1],[1,0],[-1,1],[-1,0],[-1,-1],[0,-1],[1,-1]])

        if self.lastDir[0] != 0 or self.lastDir[1] != 0:
            self.updateProb(current)

        target = pursued

        temp = tuple(current)
        temp2 = tuple(pursued)
        temp3 = tuple(pursuer)
        #pursued_plan = AnotherAnotherStar(self, world, temp2, temp3, temp)

        #for i in range(len(pursued_plan)):
        #    if i == 0:
        #        continue  
        #    elif pursued_plan[i] == temp:
        #        target = pursued_plan[1]
        #        break
        #    elif dist(pursued_plan[i], current)<dist(target, current):
        #        target = pursued_plan[i]


        our_plan = AnotherAnotherStar(self, world, temp, temp2, temp3)

        act = our_plan[1] - current



        return act
    
def dist(current, end):
    temp = abs((current[0] - end[0])^2) + abs((current[1] - end[1])^2) **0.5
    return (temp)
    



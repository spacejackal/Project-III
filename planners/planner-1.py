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

def miniminimini(world, current, pursued, pursuer, maxdepth):
    root = MimiNode(current, pursued, pursuer)
    rows, cols = len(world), len(world[0])
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1), (0,0)]
    depth = 0
    node = root
    while depth <= maxdepth:

        our_action = None
        for dir in directions:
            newNode = MimiNode(node.current + dir, node.pursued, node.pursuer, depth, node)
            #distToEnd = (abs(newNode[0] - end[0]), abs(newNode[1] - end[1]))
            if 0 <= newNode.current[0] < rows and 0 <= newNode.current[1] < cols and world[newNode.current[0]][newNode.current[1]] == 0:
                newNode.value = hurst(newNode.current, newNode.pursued, newNode.pursuer)
                #if(our_action is None or newNode.value < our_action.value):
                our_action = newNode
            if(our_action is not None):
                node.children.append(our_action)

        pursued_action = None
        for ourAction in node.children:
            for dir in directions:
                newNode = MimiNode(ourAction.current, ourAction.pursued+dir, ourAction.pursuer, depth, ourAction)
                if 0 <= newNode.current[0] < rows and 0 <= newNode.current[1] < cols and world[newNode.current[0]][newNode.current[1]] == 0:
                    newNode.value = hurst(newNode.pursued, newNode.pursuer, newNode.current)
                    #if(pursued_action is None or newNode.value < pursued_action.value):
                    pursued_action = newNode
                if(pursued_action is not None):
                    ourAction.children.append(pursued_action)

        pursuer_action = None
        for pursuedAction in our_action.children:
            for dir in directions:
                newNode = MimiNode(pursuedAction.current, pursuedAction.pursued, pursuedAction.pursuer+dir, depth, pursuedAction)
                if 0 <= newNode.current[0] < rows and 0 <= newNode.current[1] < cols and world[newNode.current[0]][newNode.current[1]] == 0:
                    newNode.value = hurst(newNode.pursuer, newNode.current, newNode.pursued)
                    if(tuple(newNode.current) == tuple(newNode.pursuer)):
                        newNode.value = 9999999999999999
                        newNode.parent.value = 9999999999999999
                        newNode.parent.parent.value = 9999999999999999
                        
                    #if(pursuer_action is None or newNode.value < pursuer_action.value):
                    pursuer_action = newNode
                if(pursuer_action is not None):
                    pursuedAction.children.append(pursuer_action)
        depth += 1
        node = pursuer_action

    #backValue(root)

    return root



def Anotherminiminimini(world, current, pursued, pursuer, maxdepth):
    root = MimiNode(current, pursued, pursuer)
    rows, cols = len(world), len(world[0])
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1), (0,0)]
    depth = 0
    node = root
    while depth <= maxdepth:

        for dir in directions:
            newNode = MimiNode(node.current + dir, node.pursued, node.pursuer, depth, node)
            if 0 <= newNode.current[0] < rows and 0 <= newNode.current[1] < cols and world[newNode.current[0]][newNode.current[1]] == 0:
                node.children.append(newNode)
                for dir in directions:
                    newNode2 = MimiNode(newNode.current, newNode.pursued+dir, newNode.pursuer, depth, newNode)
                    if 0 <= newNode2.pursued[0] < rows and 0 <= newNode2.pursued[1] < cols and world[newNode2.pursued[0]][newNode2.pursued[1]] == 0:
                        newNode.children.append(newNode2)
                        for dir in directions:
                            newNode3 = MimiNode(newNode2.current, newNode2.pursued, newNode2.pursuer+dir, depth, newNode2)
                            if 0 <= newNode3.pursuer[0] < rows and 0 <= newNode3.pursuer[1] < cols and world[newNode3.pursuer[0]][newNode3.pursuer[1]] == 0:
                                newNode2.children.append(newNode3)
                                newNode3.value = hurst(newNode3.pursuer, newNode3.current, newNode3.pursued)
                                
                                if newNode2.bestChild == None or newNode2.bestChild.value > newNode3.value:
                                    newNode2.bestChild = newNode3
                        newNode2.value = hurst(newNode2.bestChild.pursued, newNode2.bestChild.pursuer, newNode2.bestChild.current)
                        if newNode.bestChild == None or newNode.bestChild.value > newNode2.value:
                                    newNode.bestChild = newNode2
                newNode.value = hurst(newNode.bestChild.bestChild.current, newNode.bestChild.bestChild.pursued, newNode.bestChild.bestChild.pursuer)
                if node.bestChild == None or node.bestChild.value > newNode.value:
                            node.bestChild = newNode

        depth += 1
        node =  node.bestChild.bestChild.bestChild

    #backValue(root)

    return root

def AnotherAnotherminiminimini(world, current, pursued, pursuer, maxdepth):
    root = MimiNode(current, pursued, pursuer)
    rows, cols = len(world), len(world[0])
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1), (0,0)]
    depth = 0
    node = root
    while depth <= maxdepth:

        for dir in directions:
            newNode = MimiNode(node.current + dir, node.pursued, node.pursuer, depth, node)
            if 0 <= newNode.current[0] < rows and 0 <= newNode.current[1] < cols and world[newNode.current[0]][newNode.current[1]] == 0:
                node.children.append(newNode)
                for dir in directions:
                    newNode2 = MimiNode(newNode.current, newNode.pursued+dir, newNode.pursuer, depth, newNode)
                    if 0 <= newNode2.pursued[0] < rows and 0 <= newNode2.pursued[1] < cols and world[newNode2.pursued[0]][newNode2.pursued[1]] == 0:
                        newNode.children.append(newNode2)
                        for dir in directions:
                            newNode3 = MimiNode(newNode2.current, newNode2.pursued, newNode2.pursuer+dir, depth, newNode2)
                            if 0 <= newNode3.pursuer[0] < rows and 0 <= newNode3.pursuer[1] < cols and world[newNode3.pursuer[0]][newNode3.pursuer[1]] == 0 :
                                newNode2.children.append(newNode3)
                                node3AStar = AnotherAnotherStar(world, tuple(newNode3.pursuer), tuple(newNode3.current), tuple(newNode3.pursued))
                                if node3AStar is not None:
                                    newNode3.value = len(node3AStar)
                                else:
                                    newNode3.value = 9999999999999999
                                if newNode2.bestChild == None or newNode2.bestChild.value > newNode3.value:
                                    newNode2.bestChild = newNode3
                        node2Astar = AnotherAnotherStar(world, tuple(newNode2.bestChild.pursued), tuple(newNode2.bestChild.pursuer), tuple(newNode2.bestChild.current))
                        if node2Astar is not None:
                            newNode2.value = len(node2Astar)
                        else:
                            newNode2.value = 9999999999999999
                        #newNode2.value = len(AnotherAnotherStar(world, tuple(newNode2.bestChild.pursued), tuple(newNode2.bestChild.pursuer), tuple(newNode2.bestChild.current)))
                        if newNode.bestChild == None or newNode.bestChild.value > newNode2.value:
                                    newNode.bestChild = newNode2
                nodeAstar = AnotherAnotherStar(world, tuple(newNode.bestChild.bestChild.current), tuple(newNode.bestChild.bestChild.pursued), tuple(newNode.bestChild.bestChild.pursuer))
                if nodeAstar is not None:
                    newNode.value = len(nodeAstar)
                else:
                    newNode.value = 9999999999999999
                #newNode.value = len(AnotherAnotherStar(world, tuple(newNode.bestChild.bestChild.current), tuple(newNode.bestChild.bestChild.pursued), tuple(newNode.bestChild.bestChild.pursuer)))
                if node.bestChild == None or node.bestChild.value > newNode.value:
                            node.bestChild = newNode

        depth += 1
        node =  node.bestChild.bestChild.bestChild

    #backValue(root)

    return root

def getNodeVal(node:MimiNode, i):
    if not node.children:
        if(i%3 == 0):
            return hurst(node.current, node.pursued, node.pursuer)
        elif(i%3 == 1):
            return hurst(node.pursued, node.pursuer, node.current)
        else:
            return hurst(node.pursuer, node.current, node.pursued)
    
    if(i%3 == 0):
        return hurst(node.current, node.pursued, node.pursuer)
    elif(i%3 == 1):
        return hurst(node.pursued, node.pursuer, node.current)
    else:
        return hurst(node.pursuer, node.current, node.pursued)

            
def backValue(node: MimiNode):
    if node is None:
        return 0
    




def AnotherAnotherStar(grid, start, end, avoid):
    rows, cols = len(grid), len(grid[0])
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1), (0,0)]
    
    pathWeight = {start: 0}
    parent = {start: None}

    open = [start]

    found = False
    while not found and len(open) > 0:
        node = None
        for newNode in open:
            if node is None or pathWeight[newNode] <= pathWeight[node]:
                node = newNode
        
        open.remove(node)

        for dir in directions:
            newNode = (node[0] + dir[0], node[1] + dir[1])
            #distToEnd = (abs(newNode[0] - end[0]), abs(newNode[1] - end[1]))
            #distToEnd = (hurst(newNode, end,avoid))
            if 0 <= newNode[0] < rows and 0 <= newNode[1] < cols and grid[newNode[0]][newNode[1]] == 0 and newNode not in list(pathWeight.keys()):
                    #pathWeight[newNode] = 1 + pathWeight[node] + distToEnd[0] + distToEnd[1]
                    #pathWeight[newNode] = 1 + pathWeight[node] + distToEnd
                    pathWeight[newNode] = hurst(newNode, end,avoid) + pathWeight[node] + dist(start, end)
                    parent[newNode] = node
                    open.append(newNode)
                    if newNode == end:
                        found = True
                        break

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

def hurst(current, end, pursuer):
    if tuple(current) == tuple(pursuer):
        return 9999999999999999
    temp = abs((current[0] - end[0])^2) + abs((current[1] - end[1])^2) **0.5
    temp2 = abs((current[0] - pursuer[0])^2) + abs((current[1] - pursuer[1])^2) **0.5

    if temp2 == 0:
        return 9999999999999999


    return ((1/temp2) *temp)


class PlannerAgent:
    
    def __init__(self):
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
          
        target = pursued

        if dist(current, target) > 1.5 and dist(current, pursuer) > 1.5:
            temp = tuple(current)
            temp2 = tuple(pursued)
            temp3 = tuple(pursuer)
            pursued_plan = AnotherAnotherStar(world, temp2, temp3, temp)

            for i in range(len(pursued_plan)):
                if i == 0:
                    continue  
                elif pursued_plan[i] == temp:
                    target = pursued_plan[1]
                    break
                elif dist(pursued_plan[i], current)<dist(target, current):
                    target = pursued_plan[i]


            our_plan = AnotherAnotherStar(world, temp, temp2, temp3)

            act = our_plan[1] - current



            return act
        else:
            minimini = Anotherminiminimini(world, current, pursued, pursuer, 1)
            #act = minimini.children[0].current - current
            act = minimini.bestChild.current - minimini.current
            return act
    
def dist(current, end):
    temp = abs((current[0] - end[0])^2) + abs((current[1] - end[1])^2) **0.5
    return (temp)
    



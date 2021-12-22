import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import distance


class A_star:
    def __init__(self, Map):
        self.freeNodes_ = Map.freeNodes_
        self.obstructedNodes_ = Map.obstructedNodes_
        self.nodeDist_ = Map.nodeDist_
        self.worldLims_ = {"x": [Map.worldStart_[0], Map.worldXlim_], "y": [Map.worldStart_[1], Map.worldYlim_]}
        self.movements_ = self.getMovements()
        self.maxIterations_ = 1000

    def getMovements(self):
        # Add a set of possible movements
        movements = []
        for i in range(-1, 2, 1):
            for j in range(-1, 2, 1):
                movements.append((i * self.nodeDist_, j * self.nodeDist_))
        return movements

    def searchPath(self, start, end):
        print('Looking for path...')

        # Get suitable start/goal positions
        if start not in self.freeNodes_:
            closest_index = distance.cdist([start], self.freeNodes_).argmin()
            start = self.freeNodes_[closest_index]
            start_node = Node(None, start)
        else:
            start_node = Node(None, start)
        start_node.g = start_node.h = start_node.f = 0

        if end not in self.freeNodes_:
            closest_index = distance.cdist([end], self.freeNodes_).argmin()
            end = self.freeNodes_[closest_index]
            goal_node = Node(None, end)
        else:
            goal_node = Node(None, end)
        goal_node.g = goal_node.h = goal_node.f = 0

        open = [start_node]  # Start at start node
        closed = []

        iteration = 0
        while len(open) > 0 and iteration <= self.maxIterations_:
            iteration += 1
            current_node = open[0]
            for i in range(len(open)):
                if open[i].f < current_node.f or open[i].f == current_node.f:
                    if open[i].h < current_node.h:
                        current_node = open[i]

            open.remove(current_node)
            closed.append(current_node)

            # Check if goal is found
            if current_node.position == end:
                return self.getPath(current_node) # Return path and exit loop

            # Find adjacent nodes
            children = self.findAdjacent(current_node)

            # Update heuristics
            for child in children:
                for closed_child in closed:
                    if child == closed_child:
                        continue

                child.g = current_node.g + 0.5
                child.h = self.getDistance(child.position, goal_node.position)  # Heuristic function
                child.f = child.g + child.h

                # Check if child is in the open list already
                for open_node in open:
                    if child == open_node and child.g > open_node.g:
                        continue
                open.append(child)

        print("No path found")
        return None  # If no path was found

    def getDistance(self, nodeA, nodeB):
        dstX = abs(nodeA[0] - nodeB[0])
        dstY = abs(nodeA[1] - nodeB[1])
        if (dstX > dstY):
            return 14 * dstY + 10 * (dstX - dstY)
        return 14 * dstX + 10 * (dstY - dstX)

    def findAdjacent(self, currentNode):
        children = []
        for movement in self.movements_:
            new_position = (currentNode.position[0] + movement[0], currentNode.position[1] + movement[1])
            if new_position[0] <= np.min(self.worldLims_["x"]) or \
                    new_position[0] > np.max(self.worldLims_["x"]) or \
                    new_position[1] <= np.min(self.worldLims_["y"]) or \
                    new_position[1] > np.max(self.worldLims_["y"]):
                continue

            if new_position not in self.obstructedNodes_:
                new_node = Node(currentNode, new_position)
                children.append(new_node)
        return children

    def getPath(self, currentNode):
        print('end found')
        path = []
        current = currentNode
        while current is not None:
            path.append(current.position)
            current = current.parent
        return path[::-1]  # return reverse path


class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

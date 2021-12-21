import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import distance


class A_star:
    def __init__(self, Map):
        self.freeNodes_ = Map.freeNodes_
        self.obstructedNodes_ = Map.obstructedNodes_
        self.nodeDist_ = Map.nodeDist_
        self.worldLims_ = {"x": [Map.worldStart_[0], Map.worldXlim_],
                           "y": [Map.worldStart_[1], Map.worldYlim_]}
        self.movements_ = self.getMovements()

    def getMovements(self):
        # Add a set of possible movements
        movements = []
        for i in range(-1, 2, 1):
            for j in range(-1, 2, 1):
                movements.append((i * self.nodeDist_, j * self.nodeDist_))
        return movements

    def search_path(self, start, end):
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

        print('Looking for path...')
        open = []
        closed = []

        # Start at start node
        open.append(start_node)

        while len(open) > 0:
            current_node = open[0]
            for i in range(len(open)):
                if open[i].f < current_node.f or open[i].f == current_node.f:
                    if open[i].h < current_node.h:
                        current_node = open[i]

            open.remove(current_node)
            closed.append(current_node)

            # Check if goal is found
            if current_node.position == end:
                print('end found')
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    plt.scatter(current.position[0], current.position[1], color="g")
                    current = current.parent
                return path[::-1]
                # return reverse path and gets out of the loop
            children = []
            for new_position in self.movements_:
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
                if node_position[0] <= np.min(self.worldLims_["x"]) or node_position[0] > np.max(
                        self.worldLims_["x"]) or node_position[1] <= np.min(self.worldLims_["y"]) or \
                        node_position[1] > np.max(self.worldLims_["y"]):
                    continue

                if node_position not in self.obstructedNodes_:
                    new_node = Node(current_node, node_position)
                    children.append(new_node)

            for child in children:
                for closed_child in closed:
                    if child == closed_child:
                        continue

                ##create g,h,f values
                child.g = current_node.g + 0.5
                ## heuristic function used
                child.h = self.GetDistance(child.position, goal_node.position)

                child.f = child.g+child.h

                ## check if child is in the open list already
                for open_node in open:
                    if child == open_node and child.g > open_node.g:
                        continue

                open.append(child)

    def GetDistance(self, nodeA, nodeB):
        dstX = abs(nodeA[0] - nodeB[0])
        dstY = abs(nodeA[1] - nodeB[1])
        if (dstX > dstY):
            return 14 * dstY + 10 * (dstX - dstY)
        return 14 * dstX + 10 * (dstY - dstX)


class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

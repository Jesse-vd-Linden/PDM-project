
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point
from matplotlib import path

class Map:
    def __init__(self):
        self.unwalkable = []
        self.obstacles = []
        self.worldCenter = (0,0)
        self.worldx = self.worldCenter[0] + 10
        self.worldy = self.worldCenter[1] + 10
        self.nodeRadius = 0.5
        self.nodeDiameter = self.nodeRadius*2
        self.gridSizeX = int(self.worldx/self.nodeRadius)
        self.gridSizeY = int(self.worldy/self.nodeRadius)
        self.bottomLeft = self.worldCenter
        self.mapEdges = [(self.worldCenter),(0, self.worldy),(self.worldx, self.worldy), (self.worldx, 0)]
        #bottomleft,topleft ,top right,bottomright


    def findCenters(self):
        cellsCenters = []
        for i in range(self.worldx):
            for j in range(self.worldy):
                cellsCenters.append((i+self.nodeRadius, j+self.nodeRadius))
                cellsCenters.append((i, j))
                cellsCenters.append((i+self.nodeRadius, j))
                cellsCenters.append((i, j+self.nodeRadius))
        for point in cellsCenters:
            plt.scatter(point[0],point[1],color='b')
        plt.xlim([self.bottomLeft[0], self.worldx])
        plt.ylim([self.bottomLeft[1], self.worldy])
        #plt.show()
        return cellsCenters

    def setObstacles(self,shape,center,h, w=None):
        # check for the conditions
        if shape == "r":
            obstacle_temp = plt.Rectangle((center[0]-w/2, center[1]-h/2), h, w, fc='black')
            obstacle = plt.Rectangle((center[0] - w / 2, center[1] - h / 2), h + self.nodeRadius, w + self.nodeRadius,
                                     fc='black')
            plt.gca().add_patch(obstacle_temp)

        elif shape == "c":
            r = h
            obstacle_temp = plt.Circle((center[0], center[1]), r, fc='black')
            obstacle = plt.Circle((center[0], center[1]), r + self.nodeRadius, fc='black')
            plt.gca().add_patch(obstacle_temp)
        else :
            print('nope')
        self.obstacles.append(obstacle)

    def setunwalkable(self,point):
        self.unwalkable.append(point)


    def update(self):
        for point in self.unwalkable:
            plt.scatter(point[0], point[1],color="r")
##Start of Planner class############
def GetDistance(nodeA,nodeB):
    dstX = abs(nodeA[0] - nodeB[0])
    dstY = abs(nodeA[1] - nodeB[1])
    if (dstX > dstY) :
        return 14*dstY + 10* (dstX-dstY)
    return 14*dstX + 10 * (dstY-dstX)

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = self.g+self.h


    # def __eq__(self, other):
    #     return self.position == other.position





def searching(grid,start,end):
    movements = [(0, -0.5), (0, 0.5), (-0.5, 0), (0.5, 0), (-0.5, -0.5), (-0.5, 0.5), (0.5, -0.5), (0.5, 0.5)]
    start_node = Node(None,start)
    start_node.g = start_node.h = start_node.f = 0
    goal_node = Node(None, end)
    goal_node.g = goal_node.h = goal_node.f = 0
    print('Looking for path...')
    open = []
    closed = []

    open.append(start_node)

    while len(open)>0:
        current_node = open[0]
        for i in range(len(open)):
            if open[i].f < current_node.f or open[i].f == current_node.f:
                if open[i].h < current_node.h:
                    current_node = open[i];

        open.remove(current_node)
        closed.append(current_node)
        #print('while')

        #check if goal is found
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
        for new_position in movements:
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            if node_position[0]<=0 or node_position[0]>Grid.worldx or node_position[1]<=0 or node_position[1]>Grid.worldy:
                continue

            if node_position not in Grid.unwalkable:
                new_node = Node(current_node, node_position)
                children.append(new_node)
            # print(end, node_position)
        for child in children:
            for closed_child in closed:
                if child == closed_child:
                    continue
            ##create g,h,f values
            child.g = current_node.g + 0.5
            ## herustic function used
            child.h = GetDistance(child.position,goal_node.position)
                #((child.position[0] - goal_node.position[0]) ** 2) + ((child.position[1] - goal_node.position[1]) ** 2)

            ## check if child is in the open list already
            for open_node in open:
                if child == open_node and child.g > open_node.g:
                    continue

            open.append(child)


def freespace(list, cells):
    for i in range(len(list)):
        N = list[i].contains_points(cells)
        f = [i for i, x in enumerate(N) if x]
        for i in f:
            test = cells[i]
            Grid.setunwalkable(test)


if __name__ == "__main__":
    Grid = Map()
    Grid.__init__()
    cellscenters = Grid.findCenters()
    Grid.setObstacles("c",(5,3),1.1)
    Grid.setObstacles("r",cellscenters[5],3,3)
    #Grid.setObstacles("c", (8, 2), 1.3)
    freespace(Grid.obstacles,cellscenters)
    Grid.update()
    start = (6.5,6.5)
    end = (5,0.5)
    path = searching(Grid,start,end)

    print(path)
    plt.show()


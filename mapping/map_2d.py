import matplotlib.pyplot as plt
from itertools import compress
import numpy as np


class Map:
    def __init__(self, nodeDist, worldStart=(0, 0), worldSize=(10, 10)):
        self.obstructedNodes_ = []
        self.obstacles_ = []
        self.worldStart_ = worldStart
        self.worldXlim_ = self.worldStart_[0] + worldSize[0]
        self.worldYlim_ = self.worldStart_[1] + worldSize[0]
        self.nodeDist_ = nodeDist
        self.freeNodes_ = self.findCenters()

    def findCenters(self):
        cellsCenters = []
        for x in np.arange(self.worldStart_[0] - self.nodeDist_ / 2, self.worldXlim_, self.nodeDist_):
            for y in np.arange(self.worldStart_[1] - self.nodeDist_ / 2, self.worldYlim_, self.nodeDist_):
                cellsCenters.append((x, y))
        return cellsCenters

    def setObstacle(self, centerPosition, radius, w=None):
        # Add obstacle
        obstacle = plt.Circle((centerPosition[0], centerPosition[1]), radius, fc='black')
        self.obstacles_.append(obstacle)

        # Check collision with robot
        collision_obstacle = plt.Circle((centerPosition[0], centerPosition[1]), radius + self.nodeDist_, fc='black')
        point_in_collision = collision_obstacle.contains_points(self.freeNodes_).tolist()

        # Add obstructed nodes to list
        obstructedNodes = list(compress(self.freeNodes_, point_in_collision))
        self.obstructedNodes_ += obstructedNodes

        # Remove obstructed nodes from free Nodes
        for node in obstructedNodes:
            self.freeNodes_ = list(filter(lambda i: i != node, self.freeNodes_))

    def plotMap2d(self):
        # Plot cell centers
        for point in self.freeNodes_:
            plt.scatter(point[0], point[1], color='b')
        plt.xlim([self.worldStart_[0], self.worldXlim_])
        plt.ylim([self.worldStart_[1], self.worldYlim_])

        # Plot obstacles
        for obstacle in self.obstacles_:
            plt.gca().add_patch(obstacle)

        # Plot unwalkable points
        for point in self.obstructedNodes_:
            plt.scatter(point[0], point[1], color="r")

import matplotlib.pyplot as plt
from itertools import compress
import numpy as np


class Map:
    def __init__(self, nodeDist, worldStart=(0, 0), worldSize=(10, 10)):
        self.unwalkable_ = []
        self.obstacles_ = []
        self.worldStart_ = worldStart
        self.worldXlim_ = self.worldStart_[0] + worldSize[0]
        self.worldYlim_ = self.worldStart_[1] + worldSize[0]
        self.nodeDist_ = nodeDist
        self.cellsCenters_ = self.findCenters()

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
        point_in_collision = collision_obstacle.contains_points(self.cellsCenters_).tolist()
        self.unwalkable_ += list(compress(self.cellsCenters_, point_in_collision))

    def plotMap2d(self):
        # Plot cell centers
        for point in self.cellsCenters_:
            plt.scatter(point[0], point[1], color='b')
        plt.xlim([self.worldStart_[0], self.worldXlim_])
        plt.ylim([self.worldStart_[1], self.worldYlim_])

        # Plot obstacles
        for obstacle in self.obstacles_:
            plt.gca().add_patch(obstacle)

        # Plot unwalkable points
        for point in self.unwalkable_:
            plt.scatter(point[0], point[1], color="r")

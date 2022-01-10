from itertools import compress

import matplotlib.pyplot as plt
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
        self.foundPath_ = None
        self.Robot_ = Robot(armLength=1)

    def findCenters(self):
        cellsCenters = []
        for x in np.arange(self.worldStart_[0] + self.nodeDist_ / 2, self.worldXlim_, self.nodeDist_):
            for y in np.arange(self.worldStart_[1] + self.nodeDist_ / 2, self.worldYlim_, self.nodeDist_):
                cellsCenters.append((x, y))
        return cellsCenters

    def setObstacle(self, centerPosition, radius, w=None):
        # Add obstacle
        obstacle = plt.Circle((centerPosition[0], centerPosition[1]), radius, fc='black')
        self.obstacles_.append(((centerPosition[0], centerPosition[1]), radius))

        # Check collision with robot
        collision_obstacle = plt.Circle((centerPosition[0], centerPosition[1]), radius + self.Robot_.baseRadius_,
                                        fc='black')
        point_in_collision = collision_obstacle.contains_points(self.freeNodes_).tolist()

        # Add obstructed nodes to list
        obstructedNodes = list(compress(self.freeNodes_, point_in_collision))
        self.obstructedNodes_ += obstructedNodes

        # Remove obstructed nodes from free Nodes
        for node in obstructedNodes:
            self.freeNodes_ = list(filter(lambda i: i != node, self.freeNodes_))

    def addPathToMap(self, path):
        self.foundPath_ = path

    def plotMap2d(self):
        # Plot cell centers
        for point in self.freeNodes_:
            plt.scatter(point[0], point[1], color='b')
        plt.xlim([self.worldStart_[0], self.worldXlim_])
        plt.ylim([self.worldStart_[1], self.worldYlim_])

        # Plot obstacles
        for obstacle in self.obstacles_:
            circle = plt.Circle(obstacle[0], obstacle[1], fc='black')
            plt.gca().add_patch(circle)

        # Plot unwalkable points
        for point in self.obstructedNodes_:
            plt.scatter(point[0], point[1], color="red")

        # Plot path if path is added to map
        if self.foundPath_ is not None:
            plt.plot(*zip(*self.foundPath_), color="g")
            for position in self.foundPath_:
                plt.scatter(position[0], position[1], color="g")
        plt.show()

    def plotGrid3D(self):
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        # Plot cell centers
        for point in self.freeNodes_:
            ax.scatter3D(point[0], point[1], 0, color='b', s=1)
        ax.set_xlim3d([self.worldStart_[0], self.worldXlim_])
        ax.set_ylim3d([self.worldStart_[1], self.worldYlim_])
        ax.set_zlim3d([0, 10])

        # Plot obstacles
        for obstacle in self.obstacles_:
            self.plotCylinder(ax, obstacle[0][0], obstacle[0][1], obstacle[1], 3)

        # Plot unwalkable points
        for point in self.obstructedNodes_:
            ax.scatter3D(point[0], point[1], 0, color="r", s=1)

        return ax

    def plotMap3d(self):
        ax = self.plotGrid3D()
        # Plot interactively if path is added to map
        if self.foundPath_ is not None:
            ax.plot(*zip(*self.foundPath_), color="g")

            # Plot robot on path starting point
            self.Robot_.setBasePosition(self.foundPath_[0])
            self.plotRobot3D(ax=ax)
        else:
            self.plotGrid3D()
        plt.show()


    def plotCylinder(self, ax, center_x, center_y, radius, height_z):
        z = np.linspace(0, height_z, 50)
        theta = np.linspace(0, 2 * np.pi, 50)
        theta_grid, z_grid = np.meshgrid(theta, z)
        x_grid = radius * np.cos(theta_grid) + center_x
        y_grid = radius * np.sin(theta_grid) + center_y
        ax.plot_surface(x_grid, y_grid, z_grid, color="grey")

    def plotRobot3D(self, ax, arm_position=((0, 0, 0.5), (0, -1, 0.5)), size=(1, 1, 1)):
        x, y = self.Robot_.getBasePosition()
        self.plotCylinder(ax, x, y, self.Robot_.baseRadius_, self.Robot_.baseHeight_)
        armCoordinates = self.Robot_.getArmCoordinates()
        ax.plot(armCoordinates[:, 0], armCoordinates[:, 1], armCoordinates[:, 2], color="r", linewidth=5)


class Robot:
    def __init__(self, armLength=0.5):
        self.armLength_ = armLength
        self.position_ = np.array([0, 0, 0])
        self.baseRadius_ = 0.5
        self.baseHeight_ = 0.5
        self.baseRTheta_ = np.pi / 3  # Rotation of base Around Z-axis
        self.arm1Phi_ = 0  # Rotation of the first joint
        self.arm2Psi_ = np.pi / 5  # Rotation of the second joint

    def setBasePosition(self, position):
        self.position_ = np.array([position[0], position[1], 0])

    def getBasePosition(self):
        return (self.position_[0], self.position_[1])

    def getArmCoordinates(self):
        first_node = self.position_ + np.array([0, 0, self.baseHeight_])
        second_node = first_node + np.array([0, 0, self.armLength_])
        third_node = second_node + np.array([0, self.baseRadius_, self.armLength_])

        return np.array([first_node, second_node, third_node])

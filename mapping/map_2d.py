from matplotlib import path
import matplotlib.pyplot as plt

class Map:
    def __init__(self, nodeRadius):
        self.unwalkable_ = []
        self.obstacles_ = []
        self.cellsCenters_ = self.findCenters()
        self.worldCenter_ = (0, 0)
        self.worldx_ = self.worldCenter_[0] + 10
        self.worldy_ = self.worldCenter_[1] + 10
        self.nodeRadius_ = nodeRadius
        self.nodeDiameter_ = self.nodeRadius_ * 2
        self.gridSizeX_ = int(self.worldx_ / self.nodeRadius_)
        self.gridSizeY_ = int(self.worldy_ / self.nodeRadius_)
        self.bottomLeft_ = self.worldCenter_
        self.mapEdges_ = [(self.worldCenter_), (0, self.worldy_), (self.worldx_, self.worldy_), (self.worldx_, 0)]

    def findCenters(self):
        cellsCenters = []
        for i in range(self.worldx_):
            for j in range(self.worldy_):
                cellsCenters.append((i + self.nodeRadius_, j + self.nodeRadius_))
                cellsCenters.append((i, j))
                cellsCenters.append((i + self.nodeRadius_, j))
                cellsCenters.append((i, j+self.nodeRadius_))
        return cellsCenters

    def setObstacles(self,shape,center,h, w=None):
        # check for the conditions
        if shape == "r":
            obstacle_temp = plt.Rectangle((center[0]-w/2, center[1]-h/2), h, w, fc='black')
            obstacle = plt.Rectangle((center[0] - w / 2, center[1] - h / 2), h + self.nodeRadius_, w + self.nodeRadius_,
                                     fc='black')
            plt.gca().add_patch(obstacle_temp)

        elif shape == "c":
            r = h
            obstacle_temp = plt.Circle((center[0], center[1]), r, fc='black')
            obstacle = plt.Circle((center[0], center[1]), r + self.nodeRadius_, fc='black')
            plt.gca().add_patch(obstacle_temp)
        else :
            print('nope')
        self.obstacles_.append(obstacle)

    def setunwalkable(self,point):
        self.unwalkable_.append(point)


    def update(self):
        for point in self.unwalkable_:
            plt.scatter(point[0], point[1],color="r")

    def plotMap2d(self):
        for point in self.cellsCenters_:
            plt.scatter(point[0], point[1], color='b')
        plt.xlim([self.bottomLeft_[0], self.worldx_])
        plt.ylim([self.bottomLeft_[1], self.worldy_])
        plt.show()
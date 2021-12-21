from mapping.map_2d import Map
from pathfinding.a_star import *
import matplotlib.pyplot as plt

if __name__ == "__main__":
    Grid = Map(nodeDist=.5)
    Grid.setObstacle(centerPosition=(5, 3), radius=1.3)
    Grid.setObstacle(centerPosition=(0, 0), radius=3)
    Grid.plotMap2d()

    start = (5, 10)
    end = (5, 0.5)
    pathPlanner = A_star(Grid)
    pathPlanner.search_path(start, end)
    plt.show()

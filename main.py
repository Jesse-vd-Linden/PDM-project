from mapping.map_2d import Map
from pathfinding.a_star import A_star

if __name__ == "__main__":
    Grid = Map(nodeDist=.5)
    Grid.setObstacle(centerPosition=(5, 3), radius=1.3)
    Grid.setObstacle(centerPosition=(0, 0), radius=3)
    Grid.setObstacle(centerPosition=(9, 6), radius=1.8)

    start = (5, 10)
    end = (10, 0.5)
    pathPlanner = A_star(Grid)
    path = pathPlanner.searchPath(start, end)
    Grid.addPathToMap(path)
    Grid.plotMap3d()

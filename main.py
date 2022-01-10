from mapping.map_2d import Map
from pathfinding.a_star import A_star

if __name__ == "__main__":
    Grid = Map(nodeDist=0.5)
    Grid.setObstacle(centerPosition=(6, 6), radius=2)
    Grid.setObstacle(centerPosition=(8.5, 2), radius=0.5)
    Grid.setObstacle(centerPosition=(9, 0), radius=1.7)
    Grid.setObstacle(centerPosition=(2, 3), radius=1.5)

    start = (0, 10)
    end = (6, 2)
    pathPlanner = A_star(Grid)
    path = pathPlanner.searchPath(start, end)
    Grid.addPathToMap(path)
    Grid.plotMap3d()

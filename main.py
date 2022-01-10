from a_star.mapping.map import Map
from a_star.pathfinding.a_star import A_star
from rrt.RRT import RRT_arm

if __name__ == "__main__":
    # A* general path planning
    # print("Starting A* robot base path planner")
    # Grid = Map(nodeDist=0.5)
    # Grid.setObstacle(centerPosition=(6, 6), radius=2)
    # Grid.setObstacle(centerPosition=(8.5, 2), radius=0.5)
    # Grid.setObstacle(centerPosition=(9, 0), radius=1.7)
    # Grid.setObstacle(centerPosition=(2, 3), radius=1.5)
    #
    # start = (0, 10)
    # end = (6, 2)
    # pathPlanner = A_star(Grid)
    # path = pathPlanner.searchPath(start, end)
    # Grid.addPathToMap(path)
    # Grid.plotMap3d()

    # RRT manipulator path planning
    print("Starting RRT mobile manipulator planner")
    robot_arm = RRT_arm(lower_arm_length=0.3, upper_arm_length=0.3, height_base=0.1)
    path_found = robot_arm.RRT_plot(amount_nodes=6000, amount_obstacles=4)

    if path_found:
        robot_arm.simulation_arm_control()

    robot_arm.obstacle_configuration_map()

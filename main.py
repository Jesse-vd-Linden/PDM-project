from mapping.map_2d import Map
import matplotlib.pyplot as plt

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
            if node_position[0]<=0 or node_position[0]>Grid.worldx_ or node_position[1]<=0 or node_position[1]>Grid.worldy_:
                continue

            if node_position not in Grid.unwalkable_:
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
    Grid.__init__(0.5)
    cellscenters = Grid.cellsCenters_
    Grid.plotMap2d()
    Grid.setObstacles("c",(5,3),1.1)
    Grid.setObstacles("r",cellscenters[5],3,3)
    freespace(Grid.obstacles_, cellscenters)
    Grid.update()
    start = (6.5,6.5)
    end = (5,0.5)
    path = searching(Grid,start,end)

    print(path)
    plt.show()


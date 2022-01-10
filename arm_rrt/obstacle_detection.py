import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D



#### adds figure and axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.xlabel('x')
plt.ylabel('y')


##### set the world where the arm will be moving aka where RRT is used



#### this function takes the world and a number of obstacles and randomly spawn them
def set_obs(data,ob_num):
    # control colour and opacity
    alpha = 0.1
    color = [1, 0, 0, alpha] # red

# Voxels is used to customizations of the sizes, positions and colors.

    for c in range(data.shape[0]):
        # first set the world empty without obstacles
        data[c] = False

    o=0
    limits = []
    while o < ob_num:
        #for every obstacles generate 3 random ints
        obx = np.random.randint(0, data.shape[0])
        oby = np.random.randint(0, data.shape[1])
        obz = np.random.randint(0, data.shape[2])
        # set the data at those positions as true
        data[obx,oby,obz] = True
        o+=1
        # append to limits the lowest x,y,z and the highest x,y,z (aka edges of the cube obstacle)
        limits.append((obx, oby, obz))
        limits.append((obx+1, oby+1, obz+1))
        # limits are 3 tulips list each two elements after each other correspond to the edges of one obstacle
        # eg, for 1 obstacle limits will have length of 2 elements each is a 3 element tulip

    ## voxels draw the set elements of data as true into cubes in the world
    ax.voxels(data, facecolors=color, edgecolors='grey')
    ## return the list of limits
    return limits


### walkable (or obstacle detection takes the points as a list
def walkable(points, limits):
    #takes the points and gatehr xs, ys and zs together
    pointx = (points[0][0], points[1][0])
    pointy = (points[0][1], points[1][1])
    pointz = (points[0][2], points[1][2])
    #make points representing a line
    xx = np.linspace(pointx[0], pointx[1], 50)
    yy = np.linspace(pointy[0], pointy[1], 50)
    zz = np.linspace(pointz[0], pointz[1], 50)

    i=0
    while i < len(limits):
        #for every obstacle
        for j in range(50):
            x = xx[j]
            y = yy[j]
            z = zz[j]
            #check if the xs,ys and zs, of each point on the line are within the edges of the cube obstacle
            #added 0.2 as safety factor around the obstacle (change this as you find fit)
            #condition checks both lower and upper limit of each obstacle
            condition = x>=limits[i][0]-0.2 and x<=limits[i+1][0]+0.2 \
                        and y>=limits[i][1]-0.2 and y<=limits[i+1][1]+0.2  \
                        and z>=limits[i][2]-0.2 and z<=limits[i+1][2]+0.2

            if (condition):
                print('obstacle in the way')
                #return false
                return False
        #since the limits are given in pairs for each obstacle (upper limits and lower limits) we jump two
        i +=2
    #if no obstacles in the way returns true
    return True


########## main ###################
w = world_set(10)
#### replace 10 here with the size of the world at the arm movement position (where you want the obstacles to be)
d = set_obs(w,3)
#### set the number of obstacles you want it will generate randomly in the world you set before


#### points to test the functions
points = [(5,0,3),(5,10,3)]
pointx = (points[0][0], points[1][0])
pointy = (points[0][1], points[1][1])
pointz = (points[0][2], points[1][2])

xx= np.linspace(pointx[0],pointx[1],50)
yy= np.linspace(pointy[0],pointy[1],50)
zz= np.linspace(pointz[0],pointz[1],50)

#### plot the line between those points to see it in the plot for testing
ax.plot(xx,yy,zz)

#checking if the line is in freespace or not
s=walkable(points,d)
print('freespace?',s)

plt.show()
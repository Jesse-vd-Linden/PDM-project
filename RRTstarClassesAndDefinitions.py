# -*- coding: utf-8 -*-
"""
Created on Wed Dec 15 15:28:18 2021

@author: chris
"""
import numpy as np
import math
import random
import pygame

class RRTMap:
    """Class for storing the map"""
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.maph, self.mapw = self.MapDimensions
        
        # Settings for the window
        self.MapWindowName= 'Press "s" to start the simulation'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.mapw,self.maph))
        self.map.fill((255,255,255)) # Fill the map with white background
        self.nodeRad = 2
        self.nodeThickness = 0
        self.edgeThickness = 1
        
        self.obstacles = []
        self.obsdim = obsdim
        self.obsNumber = obsnum
                
        # Colors
        self.grey = (70, 70, 70)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.white = (255, 255, 255)
        self.black = (0, 0, 0)
        
    def drawMap(self, obstacles):
        pygame.draw.circle(self.map,self.green, self.start, self.nodeRad+5,0)
        pygame.draw.circle(self.map,self.green, self.goal, self.nodeRad+20,1)
        self.drawObs(obstacles)

    def drawPath(self, path):
        for n in path: # Loop through all nodes in the path
            pygame.draw.circle(self.map, self.red, n, 3, 0 )
            # Draw the disired path with a bigger node radius.
    
    def drawObs(self, obstacles):
        obstaclesList = obstacles.copy()
        while (len(obstaclesList)>0):
            obstacle = obstaclesList.pop(0)
            pygame.draw.rect(self.map,self.grey,obstacle)
    
class RRTGraph:
    """Class for storing the RRT graph"""
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        (x,y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.maph, self.mapw = MapDimensions
        self.x = []
        self.y = []
        self.parent = []
        # intializing the tree
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)
        # initialize the obstacles
        self.obstacles = []
        self.obsDim = obsdim
        self.obsNum = obsnum
        # path
        self.goalstate = None
        self.path = []
        # In between positions of the robot arm, so in the elbow
        self.x_between = []
        self.y_between = []
        # Boolian value which represents if arm or added egde crosses an object
        self.object_crossed_arm_or_edge = None
        
    def makeRandomRect(self):
        uppercornerx = int(random.uniform(0,self.mapw - self.obsDim))
        uppercornery = int(random.uniform(0,self.maph - self.obsDim))

        return (uppercornerx, uppercornery)
        
    def makeobs(self):
        obs = []
        
        for i in range(0, self.obsNum):
            rectang = None
            startgoalcol = True
            while startgoalcol:
                upper = self.makeRandomRect()
                rectang = pygame.Rect(upper,(self.obsDim,self.obsDim))
                if rectang.collidepoint(self.start) or rectang.collidepoint(self.goal):
                    startgoalcol = True
                else:
                    startgoalcol = False
            obs.append(rectang)
        self.obstacles = obs.copy()
        return obs
                
    def add_node(self, n, x, y):
        self.x.insert(n, x)
        self.y.append(y)
        
    def remove_node(self, n):
        self.x.pop(n)
        self.y.pop(n)
    
    def add_edge(self, parent, child):
        self.parent.insert(child, parent)
    
    def remove_edge(self, n):
        self.parent.pop(n)
          
    def number_of_nodes(self):
        return len(self.x)
    
    def get_distance(self, n1, n2):
        (x1, y1) = self.x[n1], self.y[n1]
        (x2, y2) = self.x[n2], self.y[n2]
        px = (float(x1) - float(x2))**2
        py = (float(y1) - float(y2))**2
        return (px+py)**(0.5)
    
    def sample_envir(self):
        valid_points = False 
        precision = 1000
        R1 = self.maph * 0.4
        R2 = self.maph * 0.4
        print(R1,R2)
        while valid_points == False:
            phi = random.randint(0,precision) / precision * np.pi
            psi = random.randint(0,precision) / precision * np.pi*2 
            x = R1 * np.cos(phi) + R2 * np.cos(phi+psi) 
            y = R1 * np.sin(phi) + R2 * np.sin(phi+psi) 
            x_bet = R1 * np.cos(phi)
            y_bet = R1 * np.sin(phi) 
            if y > 0: # and y_bet> 0:
                valid_points = True
                x_plot = -x + self.maph
                y_plot = -y + 0.9 * self.maph  
                self.x_between = -x_bet + self.maph
                self.y_between = -y_bet + 0.9 * self.maph
                x_betw = -x_bet + self.maph
                y_betw = -y_bet + 0.9 * self.maph               
                print(phi,psi,x,y,x_bet,y_bet)
 
        #x = int(random.uniform(0, self.mapw))
        #y = int(random.uniform(0, self.maph))
        #print("sample_envir stuff:")
        #print(x_plot, y_plot)
        #print(x_between, y_between)
        return x_plot, y_plot, x_betw, y_betw
    
    def nearest(self,n):
        smallest_distance = self.get_distance(0,n) # Calculate distance from new node to start node
        nearest_node_ID = 0 # Assign node ID
        for i in range(0,n): # Loop through all nodes
            if self.get_distance(i,n) < smallest_distance: # Check for all distances
                smallest_distance = self.get_distance(i,n) # Assign smallest distance
                nearest_node_ID = i # Assign node ID
        return nearest_node_ID
    
    def isFree(self):
        n = self.number_of_nodes() - 1
        (x,y) = (self.x[n], self.y[n])
        obs = self.obstacles.copy()
        while len(obs)>0:
            rectang = obs.pop(0)
            if rectang.collidepoint(x,y):
                self.remove_node(n)
                return False
        return True
            
    def crossObstacle(self, x1, x2, y1, y2):
        obs = self.obstacles.copy()
        x_between = self.x_between
        y_between = self.y_between
        x_start = self.maph
        y_start = 0.9*self.maph
        x_end = x2
        y_end = y2 
        while(len(obs)>0):
            rectang = obs.pop(0)
            for i in range(0, 101):
                    u = i/100
                    # Check between the two nodes: the edge
                    x = x1*u + x2*(1-u)
                    y = y1*u + y2*(1-u)
                    # Check lower arm
                    x_lower = x_start*u + x_between*(1-u)
                    y_lower = y_start*u + y_between*(1-u)
                    # Check upper arm
                    x_higher = x_between*u + x_end*(1-u)
                    y_higher = y_between*u + y_end*(1-u)
                    if rectang.collidepoint(x,y) or rectang.collidepoint(x_lower,y_lower) or rectang.collidepoint(x_higher,y_higher) : # Checking if the edge between nodes violates an obstacle.
                        return True
        return False
                    
    def connect(self, n1, n2):
        (x1,y1) = (self.x[n1], self.y[n1])
        (x2,y2) = (self.x[n2], self.y[n2])
        if self.crossObstacle(x1,x2,y1,y2):
            #print("do we get here")
            self.object_crossed_arm_or_edge = True
            #print(self.object_crossed_arm_or_edge)
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1, n2)
            self.object_crossed_arm_or_edge = False
            return True
        
    def step(self, nearest_node, step_node, max_distance=35):
        distance = self.get_distance(nearest_node, step_node) # calculate distance between nodes
        if distance > max_distance: # Node is too far away
            surplus_factor = max_distance/distance # this is maybe not needed?
            (x_nearest_node,y_nearest_node) = (self.x[nearest_node], self.y[nearest_node])
            (x_step_node,y_step_node) = (self.x[step_node], self.y[step_node])
            x_dif = x_step_node - x_nearest_node 
            y_dif = y_step_node - y_nearest_node 
            alpha = math.atan2(y_dif,x_dif)
            x = int( x_nearest_node + max_distance * math.cos(alpha) ) 
            y = int( y_nearest_node + max_distance * math.sin(alpha) )
            self.remove_node(step_node) # remove node with a too high distance
            if abs(x-self.goal[0]) < max_distance and abs(y-self.goal[1] < max_distance):
                self.add_node(step_node, self.goal[0], self.goal[1]) # Step to the goal
                self.goalstate = step_node # set final node to goal state
                self.goalFlag = True # We reached the goal!
            else: # if not within goal region just add the node
                self.add_node(step_node, x, y) 
                
#    def step_close(self, nearest_node, step_node, max_distance=100):
#        distance = self.get_distance(nearest_node, step_node) # calculate distance between nodes
#        if distance > max_distance:
#            a =1
#        return a
                   
    def path_to_goal(self):
        if self.goalFlag: # Check if goal has been found
            self.path = []  # Create empty list for the path
            self.path.append(self.goalstate) # Add goalstate to the path
            new_position = self.parent[self.goalstate] # get parent of goalstate 
            while(new_position != 0):
                self.path.append(new_position)
                new_position = self.parent[new_position]
            self.path.append(0)
        return self.goalFlag
        
    def getPathCoords(self):
        path_coordinates = [] # Empty list for path coordinates
        for n in self.path: # Loop through nodes in path
            x = self.x[n]
            y = self.y[n]
            path_coordinates.append((x,y)) # Add the coordinates as tuple to the list
        return path_coordinates
    
    def bias(self, goal_node): # Used to bias the algorithm to go towards the goal, (Heuristic)
        n = self.number_of_nodes()
        self.add_node(n, goal_node[0], goal_node[1]) # Add goal node to tree
        nearest_node = self.nearest(n) # Get the nearest node to goal
        self.step(nearest_node, n) # Take a step in direction of the goal
        # the step above will automatically remove the goal node when it has stepped
        self.connect(nearest_node, n)
        return self.x, self.y, self.parent
        
    
    def expand(self): # Just explore, no heuristic
        n = self.number_of_nodes()
        x, y, x_between, y_between = self.sample_envir()
        self.add_node(n, x, y)
        
#        max_dist = 100
#        nearest_node = self.nearest(n)
#        distance = self.get_distance(nearest_node, n)        
#        while distance > max_dist:
#            self.remove_node(n)
#            x, y, x_between, y_between = self.sample_envir()
#            self.add_node(n, x, y)
#            nearest_node = self.nearest(n)
#            distance = self.get_distance(nearest_node, n)
        
        if self.isFree():
            print("do we get here?")
            x_nearest = self.nearest(n)
            self.step(x_nearest, n)
            self.connect(x_nearest, n)
        return self.x, self.y, self.parent, x_between, y_between
            
    
    
    def cost(self):
        pass
    
        
        
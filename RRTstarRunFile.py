# -*- coding: utf-8 -*-
"""
Created on Wed Dec 15 15:29:02 2021

This code has been inpired by the following references:
    - https://www.youtube.com/watch?v=TzfNzqjJ2VQ&ab_channel=Algobotics
    - Robot Dynamics & Control (RO47001); Assignment 2; Luka Peternel
    
@author: chris
"""
import numpy as np
import pygame
import time
from RRTstarClassesAndDefinitions import RRTMap
from RRTstarClassesAndDefinitions import RRTGraph

def main():
    dimensions = (500,1000)
    start=(100,300)
    goal =(750,250)
    obsdim = 30
    obsnum = 0
    iteration = 0
#    t1 = 0
#    t1 = time.time()
    
    pygame.init()
    map = RRTMap(start, goal, dimensions, obsdim, obsnum)
    graph = RRTGraph(start, goal, dimensions, obsdim, obsnum)
    
    obstacles = graph.makeobs()
    map.drawMap(obstacles)
    

    font_title = pygame.font.Font('freesansbold.ttf', 24) # printing text font and font size
    text_title = font_title.render('RRT-* for a 2-DOF Robot arm', True, (map.grey), (map.white)) # printing text object
    textRect_title = text_title.get_rect()
    textRect_title.topleft = (10, 10) # printing text position with respect to the top-left corner of the window

    font_instructions = pygame.font.Font('freesansbold.ttf', 12) # printing text font and font size
    text_instructions =  font_instructions.render('Press "q" to quit the simulation.', True, (map.grey), (map.white)) # printing text object
    textRect_instructions = text_title.get_rect()
    textRect_instructions.topleft = (10,40) # printing text position with respect to the top-left corner of the window

    
    # wait until the start button is pressed
    run = True
    while run:
        for event in pygame.event.get(): # interrupt function
            if event.type == pygame.KEYUP:
                if event.key == ord('s'): # enter the main loop after 's' is pressed
                    run = False
                    
    run = True
    while run == True: # (graph.path_to_goal() == False ): # and iteration < 1000:
        for event in pygame.event.get(): # interrupt function
            if event.type == pygame.QUIT: # force quit with closing the window
                pygame.display.quit() 
                #run = False
            elif event.type == pygame.KEYUP:
                if event.key == ord('q'): # force quit with q button
                    pygame.quit()  # Allows for quiting simulation without crashing pygame
                    #run = False
           
         # Not needed I believe:   
#        elapsed = time.time()-t1
#        t1 = time.time()
#        if elapsed > 10:    # Check to see if the code gets stuck 
#            raise
            
        if iteration % 1 == 0:  # Check for every i-th iteration, so use 10 procent bias, or put infinitely high for pure random
            X, Y, Parent, X_between, Y_between = graph.expand()
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad, map.nodeThickness)
            pygame.draw.line(map.map, map.blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]), map.edgeThickness )
            # TEst circle
            #pygame.draw.circle(map.map, map.black, (map.maph, 0.9*map.maph), map.nodeRad+0.4*map.maph, 1)
            
            # Draw Robot arms
            if graph.object_crossed_arm_or_edge == False:
                print("GOOD one!")
                pygame.draw.line(map.map, map.black, (X[-1], Y[-1]), (X_between, Y_between), map.edgeThickness*2 )
                pygame.draw.line(map.map, map.black, (X_between, Y_between), (map.maph, 0.9*map.maph), map.edgeThickness*2 )
                pygame.display.update()
                time.sleep(0.25) # wait 0.25 secs
            
                #Erase robot arms
                pygame.draw.line(map.map, map.white, (X[-1], Y[-1]), (X_between, Y_between), map.edgeThickness*2 )
                pygame.draw.line(map.map, map.white, (X_between, Y_between), (map.maph, 0.9*map.maph), map.edgeThickness*2 )
            
        else: # If i-th iteration above is set to 1, we will only use random expansion.
            X, Y, Parent = graph.bias(goal)
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad, map.nodeThickness)
            pygame.draw.line(map.map, map.blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]), map.edgeThickness )
            pygame.draw.line(map.map, map.black, (X[-1], Y[-1]), (map.maph, 0.9*map.maph), map.edgeThickness*2 )
      
        if iteration % 1 == 0:
            map.map.blit(text_title, textRect_title)
            map.map.blit(text_instructions, textRect_instructions)
            pygame.display.update()
    
        time.sleep(0.1)
        iteration += 1
    
        while (graph.path_to_goal() == True):
            for event in pygame.event.get(): # interrupt function
                if event.type == pygame.QUIT: # force quit with closing the window
                    pygame.display.quit() 
                    #run = False
                elif event.type == pygame.KEYUP:
                    if event.key == ord('q'): # force quit with q button
                        pygame.quit()  # Allows for quiting simulation without crashing pygame
                        #run = False
            map.drawPath(graph.getPathCoords()) # shows the  found path
            pygame.display.update()
            time.sleep(1) # Waits 1 sec.
        
'''       Part of Test Script     '''     
#        map.map.blit(text_title, textRect_title)
#        map.map.blit(text_instructions, textRect_instructions)
#        x,y = graph.sample_envir()
#        n = graph.number_of_nodes()
#        graph.add_node(n,x,y)
#        graph.add_edge(n-1,n)
#        x1,y1 = graph.x[n], graph.y[n]
#        x2,y2 = graph.x[n-1], graph.y[n-1]
#        if (graph.isFree()):
#            pygame.draw.circle(map.map, map.red, (graph.x[n],graph.y[n]), map.nodeRad, map.nodeThickness)
#            if not graph.crossObstacle(x1,x2,y1,y2):
#                pygame.draw.line(map.map, map.blue, (x1,y1), (x2,y2), map.edgeThickness)
#            
#        pygame.display.update()
#        time.sleep(0.001) # Add small delay into system.
'''       Part of Test Script ends here     '''     
       

if __name__ == '__main__':
    main()
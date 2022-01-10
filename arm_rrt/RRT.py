import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation
import math
from numpy.core.fromnumeric import repeat
import pandas as pd
import random
from tqdm import tqdm

class RRT_arm():

    def __init__(self, lower_arm_length, upper_arm_length, height_base, dt):
        ## Robot arm values
        self.lower_arm_length = lower_arm_length
        self.upper_arm_length = upper_arm_length
        self.height_base = height_base
        self.arm_len = lower_arm_length + upper_arm_length

        ## Dynamics values
        self.start_config = [0, math.pi/1.5, -math.pi/1.5]
        self.theta = self.start_config[0]
        self.phi = self.start_config[1]
        self.psi = self.start_config[2]
        self.prev_error = 0
        self.dt = dt

        ## RRT values
        self.translation = np.array([0.6,0.6,0])
        self.connecting_radius = 0.10
        self.point_list = np.empty((0,3))
        self.line_list = np.empty((0,2,3))
        self.begin_configuration = self.position_arm()
        self.origin = np.array([0,0,0])
        self.point_list = np.append(self.point_list, np.array([self.begin_configuration]),axis=0)
        self.path = np.empty((0,3))
        self.empty_point = np.array([self.begin_configuration,self.begin_configuration]).reshape(2,3)
        
        self.x_goal = random.uniform(-self.arm_len*0.7, self.arm_len*0.7)
        self.y_goal = random.uniform(-self.arm_len*0.7, self.arm_len*0.7)
        self.z_goal = random.uniform(0, self.arm_len*0.7+self.height_base)
        # self.goal = np.array([self.x_goal, self.y_goal, self.z_goal]) + self.translation
        self.goal = np.array([-0.4, -0.2, 0.2]) + self.translation

        self.scaling_factor = 10
        self.path_found = True

########################################################################################################
#################################### DYNAMICS AND ARM ##################################################
########################################################################################################


    def position_arm(self):
        x = np.cos(self.theta)*(self.lower_arm_length*np.cos(self.phi)+self.upper_arm_length*np.cos(self.phi+self.psi)) + self.translation[0]
        y = np.sin(self.theta)*(self.lower_arm_length*np.cos(self.phi)+self.upper_arm_length*np.cos(self.phi+self.psi)) + self.translation[1]
        z = (self.height_base+self.lower_arm_length*np.sin(self.phi)+self.upper_arm_length*np.sin(self.phi+self.psi)) + self.translation[2]
        return np.array([x,y,z])


    def inverse_kinematics(self,desired_position):
        ## x_dot = Jacobian * q_dot
        ## q_dot = Jacobian_inverse * x_dot
        J11 = -1*np.sin(self.theta)*(self.lower_arm_length*np.cos(self.phi)+self.upper_arm_length*np.cos(self.phi+self.psi))
        J21 = np.cos(self.theta)*(self.lower_arm_length*np.cos(self.phi)+self.upper_arm_length*np.cos(self.phi+self.psi))
        J31 = 0

        J12 = np.cos(self.theta)*(self.lower_arm_length*-1*np.sin(self.phi))
        J22 = np.sin(self.theta)*(self.lower_arm_length*-1*np.sin(self.phi))
        J32 = (self.lower_arm_length*np.cos(self.phi))

        J13 = np.cos(self.theta)*(self.upper_arm_length*-1*np.sin(self.phi+self.psi))
        J23 = np.sin(self.theta)*(self.upper_arm_length*-1*np.sin(self.phi+self.psi))
        J33 = self.upper_arm_length*np.cos(self.phi+self.psi)

        J = np.array([[J11,J12,J13],[J21,J22,J23],[J31,J32,J33]])
        current_position = self.position_arm()
        error = desired_position - current_position
        Kp = 4
        Kd = 0.25
        derivative_error = (error-self.prev_error)/self.dt
        x_dot = Kp*error+Kd*derivative_error
        q_dot = np.dot(np.linalg.inv(J),x_dot)
        self.prev_error = error
        return q_dot


    def draw_arm(self):
        x1 = np.cos(self.theta)*(self.lower_arm_length*np.cos(self.phi))
        y1 = np.sin(self.theta)*(self.lower_arm_length*np.cos(self.phi))
        z1 = (self.height_base+self.lower_arm_length*np.sin(self.phi))

        x2 = np.cos(self.theta)*(self.lower_arm_length*np.cos(self.phi)+self.upper_arm_length*np.cos(self.phi+self.psi))
        y2 = np.sin(self.theta)*(self.lower_arm_length*np.cos(self.phi)+self.upper_arm_length*np.cos(self.phi+self.psi))
        z2 = (self.height_base+self.lower_arm_length*np.sin(self.phi)+self.upper_arm_length*np.sin(self.phi+self.psi))


        base_link = np.array([0.0,0.0,self.height_base]) + self.translation
        lower_link = np.array([x1,y1,z1]) + self.translation
        upper_link = np.array([x2,y2,z2]) + self.translation
        
        arm_configuration = np.array([self.translation,base_link,lower_link,upper_link])
        return arm_configuration

    def simulation_arm_control(self):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

    
        path = self.find_path()
        ax.plot(path[:,0],path[:,1],path[:,2],linewidth=2,color='blue')
        ax.voxels(self.x_obs, self.y_obs, self.z_obs, self.data_obstacles, facecolors=[1, 0, 0, 0.1], edgecolors='grey')
        ax.scatter(self.goal[0],self.goal[1],self.goal[2],marker='o',s=50,color='green')


        def gen(n,path):
            for i in range(n):
                idx = int((1-i/n)*path.shape[0])-1
                desired_position = path[idx,:]
                q_dot = self.inverse_kinematics(desired_position)
                self.theta += q_dot[0]*self.dt
                self.phi += q_dot[1]*self.dt
                self.psi += q_dot[2]*self.dt

                yield np.array(self.draw_arm()).T

        def update(num, data, line):
            line.set_data(data[num,0:2,:])
            line.set_3d_properties(data[num,2,:])

        N = 1000
        data = np.array(list(gen(N,path)))
        line, = ax.plot(data[0,:].flatten(), data[1,:].flatten(), data[2,:].flatten(),marker='o')

        # Setting the axes properties
        ax.set_xlim3d([-(self.arm_len+0.01)+self.translation[0], self.arm_len+0.01+self.translation[0]])
        ax.set_xlabel('X')

        ax.set_ylim3d([-(self.arm_len+0.01)+self.translation[1], self.arm_len+0.01+self.translation[1]])
        ax.set_ylabel('Y')

        ax.set_zlim3d([0, self.arm_len+0.01])
        ax.set_zlabel('Z')
        

        ani2 = animation.FuncAnimation(fig, update, N, fargs=(data, line), interval=1000/N, blit=False)
        # ani2.save('matplot003.gif', writer='imagemagick')
        plt.show()


########################################################################################################
#################################### RRT path generation ###############################################
########################################################################################################

    def RRT(self):
        ## sample a random node
        x = random.uniform(-self.arm_len,self.arm_len)
        y = random.uniform(-self.arm_len,self.arm_len)
        z = random.uniform(0,self.arm_len+self.height_base)

        ### RRT evaluation to find closest point and make a line
        current_point = np.array([x,y,z]) + self.translation
        evaluation_list = np.array([])

        # Check the distance to all existing nodes
        for j in range(self.point_list.shape[0]):
            difference_values = current_point - self.point_list[j,:]
            evaluation_value = np.sqrt((difference_values[0]**2)+(difference_values[1]**2)+(difference_values[2]**2))
            evaluation_list = np.append(evaluation_list, evaluation_value)
        
        ## Delete nodes out of radius
        evaluation_list_cut = np.delete(evaluation_list, np.where(evaluation_list >= self.connecting_radius))
        

        if evaluation_list_cut.shape == (0,): ## If no nodes in radius of sample, return nothing
            # self.line_list = np.append(self.line_list, np.array([self.empty_point]), axis=0)
            return
        else:
            index_min = np.where(evaluation_list == min(evaluation_list_cut))
            closest_point = self.point_list[index_min]
            # check for obstacles on the new line
            if self.obstacle_detection(closest_point.flatten(),current_point) == True:
                # check for connecting distance to the goal
                if np.sqrt(((current_point-self.goal)**2).sum(-1)) <= self.connecting_radius:
                    print('Found the goal!')
                    self.point_list = np.append(self.point_list,np.array([current_point]),axis=0)
                    self.line_list = np.append(self.line_list, np.array([[current_point,self.goal]]), axis=0)
                    self.line_list = np.append(self.line_list, np.array([[closest_point[0],current_point]]), axis=0)
                    return 
                else:
                    self.point_list = np.append(self.point_list,np.array([current_point]),axis=0)
                    self.line_list = np.append(self.line_list, np.array([[closest_point[0],current_point]]), axis=0)
                    return
            else:
                # self.line_list = np.append(self.line_list, np.array([self.empty_point]), axis=0)
                return


    def RRT_star(self):
        ## sample a random node
        x = random.uniform(-self.arm_len,self.arm_len)
        y = random.uniform(-self.arm_len,self.arm_len)
        z = random.uniform(0,self.arm_len+self.height_base)

        ### RRT evaluation to find closest point and make a line
        current_point = np.array([x,y,z]) + self.translation
        evaluation_list = np.array([])

        # Check the distance to all existing nodes
        for j in range(self.point_list.shape[0]):
            difference_values = current_point - self.point_list[j,:]
            evaluation_value = np.sqrt((difference_values[0]**2)+(difference_values[1]**2)+(difference_values[2]**2))
            evaluation_list = np.append(evaluation_list, evaluation_value)
        
        ## Delete nodes out of radius
        evaluation_list_cut = np.delete(evaluation_list, np.where(evaluation_list >= self.connecting_radius))
        

        if evaluation_list_cut.shape == (0,): ## If no nodes in radius of sample, return nothing
            # self.line_list = np.append(self.line_list, np.array([self.empty_point]), axis=0)
            return
        else:

            ### Check which node within the connecting radius is closest to the starting point
            cost_to_home_list = []
            for i in range(evaluation_list_cut.shape[0]):
                cost_to_home = np.sqrt(((evaluation_list_cut[i]-self.begin_configuration)**2).sum(-1))
                cost_to_home_list.append(cost_to_home)
            
            min_index = np.argmin(cost_to_home_list)
            index_min = np.where(evaluation_list == evaluation_list_cut[min_index])
            closest_point = self.point_list[index_min]

            ### Check if Connections with other points can be made in order to create quicker paths between current point and starting point
            ## CODE
            
            # check for obstacles on the new line
            if self.obstacle_detection(closest_point.flatten(),current_point) == True:
                # check for connecting distance to the goal
                if np.sqrt(((current_point-self.goal)**2).sum(-1)) <= self.connecting_radius:
                    print('Found the goal!')
                    self.point_list = np.append(self.point_list,np.array([current_point]),axis=0)
                    self.line_list = np.append(self.line_list, np.array([[current_point,self.goal]]), axis=0)
                    self.line_list = np.append(self.line_list, np.array([[closest_point[0],current_point]]), axis=0)
                    return 
                else:
                    self.point_list = np.append(self.point_list,np.array([current_point]),axis=0)
                    self.line_list = np.append(self.line_list, np.array([[closest_point[0],current_point]]), axis=0)
                    return
            else:
                # self.line_list = np.append(self.line_list, np.array([self.empty_point]), axis=0)
                return

    def obstacle_detection(self,begin_point,end_point):
        if begin_point.shape[0] == 0:
            return True
        #takes the points and gatehr xs, ys and zs together
        pointx = (begin_point[0], end_point[0])
        pointy = (begin_point[1], end_point[1])
        pointz = (begin_point[2], end_point[2])
        #make points representing a line
        xx = np.linspace(pointx[0], pointx[1], 50)
        yy = np.linspace(pointy[0], pointy[1], 50)
        zz = np.linspace(pointz[0], pointz[1], 50)

        limits = self.limits
        safety_factor = 0.02
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
                condition = x>=limits[i][0]-safety_factor and x<=limits[i+1][0]+safety_factor \
                            and y>=limits[i][1]-safety_factor and y<=limits[i+1][1]+safety_factor  \
                            and z>=limits[i][2]-safety_factor and z<=limits[i+1][2]+safety_factor

                if (condition):
                    # print('obstacle in the way')
                    #return false
                    return False
            #since the limits are given in pairs for each obstacle (upper limits and lower limits) we jump two
            i +=2
        #if no obstacles in the way returns true
        return True

    def world_set(self):
        data_x = int((self.arm_len)*self.scaling_factor)*2
        data_y = int((self.arm_len)*self.scaling_factor)*2
        data_z = int((self.arm_len)*self.scaling_factor)
        axes = [data_x, data_y,data_z]
    # Create Data
        data = np.ones(axes)
        return data


    def set_obstacles(self, obstacle_num):
       # control colour and opacity
        alpha = 0.1
        color = [1, 0, 0, alpha] # red

        data = self.world_set()

    # Voxels is used to customizations of the sizes, positions and colors.

        for c in range(data.shape[0]):
            # first set the world empty without obstacles
            data[c] = False

        o=0
        limits = []
        while o < obstacle_num:
            #for every obstacles generate 3 random ints
            obx = random.randint(0, data.shape[0]-1)
            oby = random.randint(0, data.shape[1]-1)
            obz = random.randint(0, data.shape[2]-1)

            # set the data at those positions as true

            # adding bias for the obstacles to be near the goal
            if abs(obx + int(self.goal[0]*self.scaling_factor)) < data.shape[0]//2:
                obx += int(self.goal[0]*self.scaling_factor)
            
            if abs(oby + int(self.goal[1]*self.scaling_factor)) < data.shape[1]//2:
                oby += int(self.goal[1]*self.scaling_factor)
            
            
            ## Checking if obstacle is on the goal
            if obx < self.x_goal*self.scaling_factor and self.x_goal*self.scaling_factor < obx+1:
                continue
            elif oby < self.y_goal*self.scaling_factor and self.y_goal*self.scaling_factor < oby+1:
                continue
            elif obz < self.z_goal*self.scaling_factor and self.z_goal*self.scaling_factor  < obz+1:
                continue

            data[obx,oby,obz] = True
            
            o+=1
            # append to limits the lowest x,y,z and the highest x,y,z (aka edges of the cube obstacle)
            limits.append((obx/self.scaling_factor, oby/self.scaling_factor, obz/self.scaling_factor))
            limits.append(((obx+1)/self.scaling_factor, (oby+1)/self.scaling_factor, (obz+1)/self.scaling_factor))
            # limits are 3 tulips list each two elements after each other correspond to the edges of one obstacle
            # eg, for 1 obstacle limits will have length of 2 elements each is a 3 element tulip

        ## voxels draw the set elements of data as true into cubes in the world
        x, y, z = np.indices((data.shape[0]+1, data.shape[1]+1, data.shape[2]+1)) / self.scaling_factor
        self.limits = limits
        ## return the list of limits
        return x, y, z, data


    def obstacle_configuration_map(self):
        fig3 = plt.figure()
        ax_obs = fig3.add_subplot(projection='3d')        

        steps = 20
        dataset = np.zeros((steps,steps,steps),dtype=bool)
        for i in tqdm(range(steps)):
            theta_pos = (i/steps-1/2)*2*math.pi
            for j in range(steps):
                phi_pos = (j/steps-1/2)*2*math.pi
                for k in range(steps):
                    psi_pos = (k/steps-1/2)*2*math.pi
                    if self.check_configuration(theta_pos, phi_pos, psi_pos):
                        dataset[i,j,k] = True
                        
        print(np.count_nonzero(dataset), 20**3 - np.count_nonzero(dataset))
        theta_vox, phi_vox, psi_vox = np.indices((dataset.shape[0]+1, dataset.shape[1]+1, dataset.shape[2]+1)) * math.pi / steps
        ax_obs.voxels(theta_vox, phi_vox, psi_vox, dataset, facecolors=[1, 0, 0, 0.5], edgecolors=[0.1,0.1,0.1,0.01])

        # dataset_invert = np.invert(dataset)
        # ax_obs.voxels(theta_vox, phi_vox, psi_vox, dataset_invert, facecolors=[0, 1, 0, 0.05], edgecolors=[0.1,0.1,0.1,0.1])

        ax_obs.scatter(self.start_config[0],self.start_config[1],self.start_config[2],marker='o',s=50,color='blue', label='start')
        # ax_obs.scatter(self.goal[0],self.goal[1],self.goal[2],marker='o',s=50,color='green', label='goal')


        ax_obs.set_xlabel('Theta')
        ax_obs.set_ylabel('Phi')
        ax_obs.set_zlabel('Psi')
        
        plt.show()

    def config_based_on_position(self):
        self.goal

        x = np.cos(self.theta)*(self.lower_arm_length*np.cos(self.phi)+self.upper_arm_length*np.cos(self.phi+self.psi)) + self.translation[0]
        y = np.sin(self.theta)*(self.lower_arm_length*np.cos(self.phi)+self.upper_arm_length*np.cos(self.phi+self.psi)) + self.translation[1]
        z = (self.height_base+self.lower_arm_length*np.sin(self.phi)+self.upper_arm_length*np.sin(self.phi+self.psi)) + self.translation[2]
        return

    def check_configuration(self,theta_pos,psi_pos,phi_pos):
        self.theta = theta_pos
        self.psi = psi_pos
        self.phi = phi_pos
        arm_config = self.draw_arm()
        if (arm_config[:,2] < -0.01).any():
            return True
        elif self.obstacle_detection(arm_config[0,:], arm_config[1,:]) == False: ## Self.obstacle_detection
            return True
        elif self.obstacle_detection(arm_config[1], arm_config[2]) == False: ## Self.obstacle_detection
            return True
        elif self.obstacle_detection(arm_config[2], arm_config[3]) == False: ## Self.obstacle_detection
            return True
        else:
            return False

    def find_path(self):
        goal_index = []
        for i in range(self.line_list.shape[0]):
            if ((self.line_list[i,1,:] == self.goal).all()):
                goal_index.append(i)


        cost_list =[]
        for i in goal_index:
            path_list_temp = np.array([self.goal])
            index = i
            while (path_list_temp[-1,:] != self.begin_configuration).all():
                last_point = self.line_list[index,0,:]
                path_list_temp = np.append(path_list_temp, np.array([last_point]),axis=0)
                for j in range(self.line_list.shape[0]):
                    if ((self.line_list[j,1,:] == self.line_list[index,0,:]).all()):
                        index = j
            path_cost = 0
            for i in range(path_list_temp.shape[0]-1):
                path_cost += np.sqrt(((path_list_temp[i,:]-path_list_temp[i+1,:])**2).sum(-1))
            cost_list.append(path_cost)

        if len(cost_list) == 0:
            print('No path found!')
            self.path_found = False
            return

        path_list = np.array([self.goal])
        index = goal_index[np.where(cost_list == min(cost_list))[0][0]]
        while (path_list[-1,:] != self.begin_configuration).all():
            last_point = self.line_list[index,0,:]
            path_list = np.append(path_list_temp, np.array([last_point]),axis=0)
            for j in range(self.line_list.shape[0]):
                if ((self.line_list[j,1,:] == self.line_list[index,0,:]).all()):
                    index = j

        return path_list
    
    def RRT_plot(self,amount_nodes,amount_obstacles):
        fig2 = plt.figure()
        ax_rrt = fig2.add_subplot(projection='3d')         

        self.x_obs, self.y_obs, self.z_obs, self.data_obstacles = self.set_obstacles(amount_obstacles)
        ax_rrt.voxels(self.x_obs, self.y_obs, self.z_obs, self.data_obstacles, facecolors=[1, 0, 0, 0.1], edgecolors='grey')
   
        
        def gen(n):
            for i in range(n):
                yield self.RRT_star()

        _ = np.array(list(gen(amount_nodes)))
        data = self.line_list.T
        print(data.shape)

        def update(num):
            print(num)

            ax_rrt.plot((data[0,:,num].flatten()), (data[1,:,num].flatten()), (data[2,:,num].flatten()), linewidth=1,color='red')
                         
      
        ### Beginning position plotting
        x_init = np.array([self.begin_configuration[0]])
        y_init = np.array([self.begin_configuration[1]])
        z_init = np.array([self.begin_configuration[2]])
        ax_rrt.scatter(x_init,y_init,z_init,marker='o',s=50,color='blue')
        ax_rrt.scatter(self.goal[0],self.goal[1],self.goal[2],marker='o',s=50,color='green')
        arm_still = self.draw_arm()
        ax_rrt.plot(arm_still[:,0], arm_still[:,1], arm_still[:,2],linewidth=3,marker='o')

        # Setting the axes properties
        ax_rrt.set_xlim3d([-(self.arm_len+0.01)+self.translation[0], self.arm_len+0.01+self.translation[0]])
        ax_rrt.set_xlabel('X')

        ax_rrt.set_ylim3d([-(self.arm_len+0.01)+self.translation[1], self.arm_len+0.01+self.translation[1]])
        ax_rrt.set_ylabel('Y')

        ax_rrt.set_zlim3d([0, self.arm_len+0.01])
        ax_rrt.set_zlabel('Z')
        
        if self.path_found:
            ani1 = animation.FuncAnimation(fig2, update, frames=data.shape[2], interval=100, blit=False, repeat=False) ## interval=1000ms, fargs=(data,line),

        ## FInding and printing the shortest path between the start and goal

        # ani1.save('matplot001.gif', writer='imagemagick') ### Save the animation of the gif
        plt.show()

        self.static_RRT_plot(data)

        return self.path_found

    def static_RRT_plot(self,data):
        fig4 = plt.figure()
        ax4 = fig4.add_subplot(projection='3d')         

        ax4.voxels(self.x_obs, self.y_obs, self.z_obs, self.data_obstacles, facecolors=[1, 0, 0, 0.1], edgecolors='grey')
        print(data.shape)
                         
      
        ### Beginning position plotting
        x_init = np.array([self.begin_configuration[0]])
        y_init = np.array([self.begin_configuration[1]])
        z_init = np.array([self.begin_configuration[2]])
        ax4.scatter(x_init,y_init,z_init,marker='o',s=50,color='blue')
        ax4.scatter(self.goal[0],self.goal[1],self.goal[2],marker='o',s=50,color='green')
        arm_still = self.draw_arm()
        ax4.plot(arm_still[:,0], arm_still[:,1], arm_still[:,2],linewidth=3,marker='o')

        # Setting the axes properties
        ax4.set_xlim3d([-(self.arm_len+0.01)+self.translation[0], self.arm_len+0.01+self.translation[0]])
        ax4.set_xlabel('X')

        ax4.set_ylim3d([-(self.arm_len+0.01)+self.translation[1], self.arm_len+0.01+self.translation[1]])
        ax4.set_ylabel('Y')

        ax4.set_zlim3d([0, self.arm_len+0.01])
        ax4.set_zlabel('Z')

        
        
        if self.path_found:
            for i in range(data.shape[2]):
                ax4.plot(data[0,:,i].flatten(), data[1,:,i].flatten(),data[2,:,i].flatten(),linewidth=1,color='red')

        path = self.find_path()
        print(path.shape)
        ax4.plot(path[:,0], path[:,1], path[:,2], linewidth=3, color='blue')
        plt.show()
        
        return self.path_found


########################################################################################################


def main():
    lower = 0.3
    upper = 0.3
    height = 0.1
    dt = 0.01
    rrt_arm = RRT_arm(lower, upper, height,dt)


    

    amount_nodes = 6000
    amount_obstacles = 4
    path_found = rrt_arm.RRT_plot(amount_nodes, amount_obstacles)

    if path_found:
        rrt_arm.simulation_arm_control()

    rrt_arm.obstacle_configuration_map()


if __name__ == '__main__':
    main()
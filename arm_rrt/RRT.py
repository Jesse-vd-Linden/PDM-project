import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation
import math
import pandas as pd
import random

class RRT_arm():

    def __init__(self, lower_arm_length, upper_arm_length, height_base, dt):
        ## Robot arm values
        self.lower_arm_length = lower_arm_length
        self.upper_arm_length = upper_arm_length
        self.height_base = height_base
        self.arm_len = lower_arm_length + upper_arm_length

        ## Dynamics values
        self.theta = 0
        self.phi = math.pi/1.5
        self.psi = -math.pi/1.2
        self.prev_error = 0
        self.dt = dt

        ## RRT values
        self.connecting_radius = 0.1
        self.point_list = np.empty((0,3))
        self.begin_configuration = self.position_arm()
        self.origin = np.array([0,0,0])
        self.point_list = np.append(self.point_list, np.array([self.begin_configuration]),axis=0)

########################################################################################################
#################################### DYNAMICS AND ARM ##################################################
########################################################################################################


    def position_arm(self):
        x = np.cos(self.theta)*(self.lower_arm_length*np.cos(self.phi)+self.upper_arm_length*np.cos(self.phi+self.psi))
        y = np.sin(self.theta)*(self.lower_arm_length*np.cos(self.phi)+self.upper_arm_length*np.cos(self.phi+self.psi))
        z = (self.height_base+self.lower_arm_length*np.sin(self.phi)+self.upper_arm_length*np.sin(self.phi+self.psi))
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
        Kp = 1
        Kd = 0.1
        derivative_error = (error-self.prev_error)/self.dt
        x_dot = Kp*error+Kd*derivative_error
        q_dot = np.dot(np.linalg.inv(J),x_dot)
        self.prev_error = error
        return q_dot


    def draw_arm(self, desired_position):
        q_dot = self.inverse_kinematics(desired_position)
        self.theta += q_dot[0]*self.dt
        self.phi += q_dot[1]*self.dt
        self.psi += q_dot[2]*self.dt

        x1 = np.cos(self.theta)*(self.lower_arm_length*np.cos(self.phi))
        y1 = np.sin(self.theta)*(self.lower_arm_length*np.cos(self.phi))
        z1 = (self.height_base+self.lower_arm_length*np.sin(self.phi))

        x2 = np.cos(self.theta)*(self.lower_arm_length*np.cos(self.phi)+self.upper_arm_length*np.cos(self.phi+self.psi))
        y2 = np.sin(self.theta)*(self.lower_arm_length*np.cos(self.phi)+self.upper_arm_length*np.cos(self.phi+self.psi))
        z2 = (self.height_base+self.lower_arm_length*np.sin(self.phi)+self.upper_arm_length*np.sin(self.phi+self.psi))


        base_link = np.array([0.0,0.0,self.height_base])
        lower_link = np.array([x1,y1,z1])
        upper_link = np.array([x2,y2,z2])
        
        arm_configuration = np.array([[0,0,0],base_link,lower_link,upper_link])
        return arm_configuration

    def simulation_arm_control(self,desired_position):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        def gen(n):
            for i in range(n):
                yield np.array(self.draw_arm(desired_position)).T

        def update(num, data, line):
            line.set_data(data[num,0:2,:])
            line.set_3d_properties(data[num,2,:])

        N = 1000
        data = np.array(list(gen(N)))
        print(data.shape)
        line, = ax.plot(data[0,:].flatten(), data[1,:].flatten(), data[2,:].flatten(),marker='o')

        # Setting the axes properties
        ax.set_xlim3d([-(self.arm_len+0.01), self.arm_len+0.01])
        ax.set_xlabel('X')

        ax.set_ylim3d([-(self.arm_len+0.01), self.arm_len+0.01])
        ax.set_ylabel('Y')

        ax.set_zlim3d([0, self.arm_len+0.01])
        ax.set_zlabel('Z')

        ani2 = animation.FuncAnimation(fig, update, N, fargs=(data, line), interval=10000/N, blit=False)
        # ani.save('matplot003.gif', writer='imagemagick')
        plt.show()


########################################################################################################
#################################### RRT path generation ###############################################
########################################################################################################

    def RRT(self,i):
        ## sample a random node
        x = random.uniform(-self.arm_len,self.arm_len)
        y = random.uniform(-self.arm_len,self.arm_len)
        z = random.uniform(0,self.arm_len)

        ### RRT evaluation to find closest point and make a line
        current_point = np.array([x,y,z])
        evaluation_list = np.array([])

        for j in range(self.point_list.shape[0]):
            difference_values = current_point - self.point_list[j,:]
            evaluation_value = np.sqrt((difference_values[0]**2)+(difference_values[1]**2)+(difference_values[2]**2))
            evaluation_list = np.append(evaluation_list, evaluation_value)
        
        
        evaluation_list_cut = np.delete(evaluation_list, np.where(evaluation_list >= self.connecting_radius))
        
        if evaluation_list_cut.shape == (0,):
            return np.array([self.begin_configuration,self.begin_configuration]).reshape(2,3)
        else:
            index_min = np.where(evaluation_list == min(evaluation_list_cut))
            closest_point = self.point_list[index_min]
            self.point_list = np.append(self.point_list,np.array([current_point]),axis=0)
            return np.array([closest_point[0],current_point])
    
    def RRT_plot(self,amount_nodes):
        fig2 = plt.figure()
        ax_rrt = fig2.add_subplot(projection='3d')            
        
        def gen(n):
            for i in range(n):
                yield self.RRT(i)

        def init_func():
            point.set_data(np.array([[self.begin_configuration[0],self.begin_configuration[1]],[self.begin_configuration[0],self.begin_configuration[1]]]))
            point.set_3d_properties(np.array([[self.begin_configuration[2]],[self.begin_configuration[2]]]))

        def update(num, data, point):
            if (data[:(num+1),:,:2]).shape == (1,2,2):
                point.set_data(np.squeeze(data[:(num+1),:,:2]))
                point.set_3d_properties(np.squeeze(data[:(num+1),:,2]))
            else:
                print((data[:(num+1),:,:2]).flatten().shape)
                print((data[:(num+1),:,2]).flatten().shape)
                point.set_data((data[:(num+1),:,:2]).flatten())
                point.set_3d_properties((data[:(num+1),:,2]).flatten())
                         
        data = np.array(list(gen(amount_nodes)))
        print(data.shape)

        ### Beginning position plotting
        x_init = np.array([self.begin_configuration[0],self.begin_configuration[0]+0.01])
        y_init = np.array([self.begin_configuration[1],self.begin_configuration[1]+0.01])
        z_init = np.array([self.begin_configuration[2],self.begin_configuration[2]+0.01])
        ax_rrt.plot(x_init,y_init,z_init,marker='o',linewidth=5,color='blue')

        ## plotting RRT (STATIC)
        for i in range(data.shape[0]):
            point, = ax_rrt.plot((data[i,:,0].flatten()), (data[i,:,1].flatten()), (data[i,:,2].flatten()),linewidth=1,marker='o',color='red')
       
        # Setting the axes properties
        ax_rrt.set_xlim3d([-(self.arm_len+0.01), self.arm_len+0.01])
        ax_rrt.set_xlabel('X')

        ax_rrt.set_ylim3d([-(self.arm_len+0.01), self.arm_len+0.01])
        ax_rrt.set_ylabel('Y')

        ax_rrt.set_zlim3d([0, self.arm_len+0.01])
        ax_rrt.set_zlabel('Z')

        # ani1 = animation.FuncAnimation(fig2, update, amount_nodes, fargs=(data,point), init_func=init_func, interval=1000, blit=False) ## interval=1000ms
        # ani.save('matplot001.gif', writer='imagemagick')
        plt.show()


########################################################################################################


def main():
    lower = 0.3
    upper = 0.3
    height = 0.1
    dt = 0.01
    rrt_arm = RRT_arm(lower, upper, height,dt)


    desired_position = np.array([0.3,0.3,0.1])

    amount_nodes = 3000
    rrt_arm.RRT_plot(amount_nodes)

    # rrt_arm.simulation_arm_control(desired_position)s


if __name__ == '__main__':
    main()
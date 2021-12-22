import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation
import math
import pandas as pd

class RRT_arm():

    def __init__(self, lower_arm_length, upper_arm_length, height_base, dt):
        self.lower_arm_length = lower_arm_length
        self.upper_arm_length = upper_arm_length
        self.height_base = height_base
        self.arm_len = lower_arm_length + upper_arm_length

        self.theta = 0
        self.phi = math.pi/2
        self.psi = math.pi/2
        self.prev_error = 0
        self.dt = dt

    def position_arm(self):
        x = np.cos(self.theta)*(self.lower_arm_length*np.cos(self.phi)+self.upper_arm_length*np.cos(self.phi+self.psi))
        y = np.sin(self.theta)*(self.lower_arm_length*np.cos(self.phi)+self.upper_arm_length*np.cos(self.phi+self.psi))
        z = (self.height_base+self.lower_arm_length*np.sin(self.phi)+self.upper_arm_length*np.sin(self.phi+self.psi))
        position = np.array([x,y,z])
        return position


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
        line, = ax.plot(data[0,:].flatten(), data[1,:].flatten(), data[2,:].flatten(),marker='o')

        # Setting the axes properties
        ax.set_xlim3d([-(self.arm_len+0.01), self.arm_len+0.01])
        ax.set_xlabel('X')

        ax.set_ylim3d([-(self.arm_len+0.01), self.arm_len+0.01])
        ax.set_ylabel('Y')

        ax.set_zlim3d([0, self.arm_len+0.01])
        ax.set_zlabel('Z')

        ani = animation.FuncAnimation(fig, update, N, fargs=(data, line), interval=10000/N, blit=False)
        #ani.save('matplot003.gif', writer='imagemagick')
        plt.show()



def main():
    lower = 0.3
    upper = 0.3
    height = 0.1
    dt = 0.01
    rrt_arm = RRT_arm(lower, upper, height,dt)

    desired_position = np.array([0.3,0.3,0.1])
    rrt_arm.simulation_arm_control(desired_position)


if __name__ == '__main__':
    main()
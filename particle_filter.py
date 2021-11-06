import numpy as np
from time import time
from collections import deque
import math

PARTICLE_NUMBER = 500
    
def multiply(mu1, var1, mu2, var2, var_min = 0.001):
    if var1 <= 0: var1 = var_min 
    if var2 <= 0: var2 = var_min 
    mean = (var1*mu2 + var2*mu1) / (var1+var2)
    variance = 1 / (1/var1 + 1/var2)
    return (mean, variance)


def var(x_i, x_n):
    """
    Pass in a numpy array
    """
    diff = x_i - x_n
    x = np.dot(diff, diff)
    return x


class PFilter:
    def __init__(self, init_x, init_y):
        self.x1 = init_x
        self.y1 = init_y
        self.particles = np.array([(init_x + np.random.normal(loc=0.0, scale=75), init_y + np.random.normal(loc=0.0, scale=50)) for i in range(PARTICLE_NUMBER)])
        self.particle_indices = np.arange(PARTICLE_NUMBER)
        self.weights = np.ones(PARTICLE_NUMBER) / float(PARTICLE_NUMBER)
        
        # Sensor Variances
        self.vel_x_var = None
        self.vel_y_var = None
        self.lidar_front_var = None
        self.lidar_bot_var = None
      

    def MCL(self, odom, lidar):
        '''
        Performs one step of Monte Carlo Localization.
            1. resample particle distribution to form the proposal distribution
            2. apply the motion model
            3. apply the sensor model
            4. normalize particle weights

        This is in the critical path of code execution, so it is optimized for speed.
        '''
        #initialise particles
        ##takes samples from 
        proposed_indices = np.random.choice(self.particle_indices, size=PARTICLE_NUMBER, p=self.weights)
        proposed_distribution = self.particles[proposed_indices, :]

        #Motion Model
        ## Apply Motion model to the particles
        proposed_distribution = self.motion_model(proposed_distribution, odom)

        #Sensor Model
        ## Do scan matching and see the proposed lidar vs the real lidar
        self.sensor_model(proposed_distribution, lidar, self.weights)

        # normalize weights
        self.weights /= np.sum(self.weights)
        self.particles = proposed_distribution


    def motion_model(self, particles, action):
        """
        In this function we apply the motion from the odometry to the particles.
        We also throw in some gaussian noise to the measurement to spread out the distribution.
        """

        v_x, v_y = action
        particles[:,0] += v_x 
        particles[:,1] += v_y
        

        #Add noise
        ## Note: Not sure if the scale is right, using variance works better than sd? Review
        particles[:,0] += np.random.normal(loc=0.0, scale=abs(self.vel_x_var), size=PARTICLE_NUMBER)
        particles[:,1] += np.random.normal(loc=0.0, scale=abs(self.vel_y_var), size=PARTICLE_NUMBER)

        return particles


    def sensor_model(self, particles, observation, weights):
        """
        in this function, we do some scan matching between what the car
        actually sees vs what the particles see
        The Scan matching produces the errors in a table and the weights
        are readjusted according to the table.
        # Still need to reduce noise, kf and add gaussian noise...
        """
        particle_obs_front = 400 - particles[:,0]
        particle_obs_bot = 300 - particles[:,1]

        error_table = [[], []]
        error_table[0] = (observation[0] - particle_obs_front)**2
        error_table[1] = (observation[1] - particle_obs_bot)**2

        for i in range(0, PARTICLE_NUMBER):
            weight = error_table[0][i] + error_table[1][i]
            weight /= 2
            weight = 1/weight
            #weight = np.power(weight, 1/some_number) Squashing Factor?
            weights[i] = weight
    
    
    def expected_pose(self):
        # returns the expected value of the pose given the particle distribution
        ## Correct shape? ... 
        x = self.particles[:,0] * self.weights
        #x /= PARTICLE_NUMBER
        x = np.sum(x)
        y = self.particles[:,1] * self.weights
        #y /= PARTICLE_NUMBER
        y = np.sum(y)

        return x, y


    def update(self, odom, lidar):
        self.MCL(odom, lidar)
        x, y = self.expected_pose()
        return x, y


    def calc_pose(self, odom, lidar):
        """
        Feed in odom (v_x, v_y, var_x, var_y) and lidar(front, bot, front_var, bot_var)
        Returns the best guess of the particles as well as the proposed particles and weights
        """
        self.vel_x_var, self.vel_y_var = odom[2], odom[3]
        #self.lidar_front_var, self.lidar_bot_var = lidar[2]#lidar[2:3]

        self.x1, self.y1 = self.update(odom[0:2], lidar)


        return self.x1, self.y1, self.particles, self.weights # particles and weights for vis

        

        
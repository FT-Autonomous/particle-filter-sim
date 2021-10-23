import numpy as np
from time import time
from collections import deque

    
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


class Filter:
    def __init__(self, init_x, init_y):
        #variables at "t-1"
        self.window_size = 3
        self.x_1 = init_x
        self.y_1 = init_y

        #initialise lidar
        self.lidar_front = deque(maxlen=self.window_size)
        self.lidar_front.append(init_x)
        self.lidar_bot = deque(maxlen=self.window_size)
        self.lidar_bot.append(init_y)

        #initialise odom
        self.x_velocity = deque(maxlen=self.window_size)
        self.x_velocity.append(0.0)
        self.y_velocity = deque(maxlen=self.window_size)
        self.y_velocity.append(0.0)


    def kalman_filter(self, odom, lidar):
        # True in sim, not in real life
        dt = 1 

        #Predict Step
        v_x, v_y = odom
        self.x_velocity.append(v_x)
        self.y_velocity.append(v_y)

        #Handle x
        ## variance calculation
        ## Small Scale linearisation
        ## Could be calculated like this maybe?  (https://en.wikipedia.org/wiki/Kalman_filter#:~:text=literature.%5B31%5D%5B32%5D-,Example%20application%2C%20technical,-%5Bedit%5D)
        m, c = np.polyfit(np.arange(1, len(self.x_velocity)+1), self.x_velocity, 1)
        velocity_values = np.array(m*np.array(self.x_velocity) + c)
        xk_var = var(velocity_values, np.array(self.x_velocity))

        ## Check if v_x is correct
        xk_1 = [[self.x_1], [v_x]]
        F = [[1, dt], [0, 1]]
        F_times_xk_1 = np.matmul(F, xk_1)
        xk = F_times_xk_1[0][0] + np.random.normal(loc=0.0, scale=np.sqrt(xk_var))

        #Handle y
        ## variance calculation
        ## Small Scale linearisation
        ## Could be calculated like this maybe?  (https://en.wikipedia.org/wiki/Kalman_filter#:~:text=literature.%5B31%5D%5B32%5D-,Example%20application%2C%20technical,-%5Bedit%5D)
        m, c = np.polyfit(np.arange(1, len(self.y_velocity)+1), self.y_velocity, 1)
        velocity_values = np.array(m*np.array(self.y_velocity) + c)
        yk_var = var(velocity_values, np.array(self.y_velocity))

        ## Check if v_y is correct
        yk_1 = [[self.y_1], [v_y]]
        F = [[1, dt], [0, 1]]
        F_times_yk_1 = np.matmul(F, yk_1)
        yk = F_times_yk_1[0][0] + np.random.normal(loc=0.0, scale=np.sqrt(yk_var))


        #Measurement Step
        lidar_front, lidar_bot = lidar
        lidar_front, lidar_bot = 400 - lidar_front, 300 - lidar_bot # change into lidar times transfer matrix
        self.lidar_front.append(lidar_front)
        self.lidar_bot.append(lidar_bot)

        #Handle X
        m, c = np.polyfit(np.arange(1, len(self.lidar_front)+1), self.lidar_front, 1)
        lidar_dx = np.array(m*np.arange(1, len(self.lidar_front)+1) + c)
        lidarx_k_var = var(lidar_dx, self.lidar_front)

        lidarx_k = [[lidar_front], [0]]
        H = [1, 0]
        H_times_lidarx_k = np.matmul(H, lidarx_k)

        z_lidarx_k = H_times_lidarx_k[0] + np.random.normal(loc=0.0, scale=np.sqrt(lidarx_k_var))

        #Handle Y
        m, c = np.polyfit(np.arange(1, len(self.lidar_bot)+1), self.lidar_bot, 1)
        lidar_dy = np.array(m*np.arange(1, len(self.lidar_bot)+1) + c)
        lidary_k_var = var(lidar_dy, np.array(self.lidar_bot))

        lidary_k = [[lidar_bot], [0]]
        H = [1, 0]
        H_times_lidary_k = np.matmul(H, lidary_k)
        z_lidary_k = H_times_lidary_k[0] + np.random.normal(loc=0.0, scale=np.sqrt(lidary_k_var))

        # Correction Step
        x, x_var = multiply(xk, xk_var, z_lidarx_k, lidarx_k_var)
        y, y_var = multiply(yk, yk_var, z_lidary_k, lidary_k_var)
    
        return x, y, x_var, y_var
        

    def calc_pose(self, odom, lidar):
        self.x_1, self.y_1, self.x_1_cov, self.y_1_cov = self.kalman_filter(odom, lidar)

        return self.x_1, self.y_1


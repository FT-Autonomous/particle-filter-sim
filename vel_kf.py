import numpy as np
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


class Vel_KFilter:
    def __init__(self, init_x, init_y):
        self.window_size = 3

        self.x_velocity = deque(maxlen=self.window_size)
        self.x_velocity.append(0.0)
        self.y_velocity = deque(maxlen=self.window_size)
        self.y_velocity.append(0.0)

        self.lidar_front = deque(maxlen=self.window_size)
        self.lidar_front.append(init_x)
        self.lidar_bot = deque(maxlen=self.window_size)
        self.lidar_bot.append(init_y)

    def kalman_filter(self, odom, lidar):
        dt = 1

        
        # Predict Step
        ## Set up Lidar
        lidar_front, lidar_bot = lidar
        lidar_front, lidar_bot = 400 - lidar_front, 300 - lidar_bot # change into lidar times transfer matrix
        self.lidar_front.append(lidar_front)
        self.lidar_bot.append(lidar_bot)

        ## Handle x_var
        m, c = np.polyfit(np.arange(1, len(self.lidar_front)+1), self.lidar_front, 1)
        lidar_dx = np.array(m*np.arange(1, len(self.lidar_front)+1) + c)
        lidarx_k_var = var(lidar_dx, self.lidar_front)        

        # x lidar model
        ds = lidar_front - self.lidar_front[-1]
        lidar_vx = ds / dt

        ## Handle x_var
        m, c = np.polyfit(np.arange(1, len(self.lidar_bot)+1), self.lidar_bot, 1)
        lidar_dy = np.array(m*np.arange(1, len(self.lidar_bot)+1) + c)
        lidary_k_var = var(lidar_dy, self.lidar_bot)        

        # x lidar model
        ds = self.lidar_bot[-1] - lidar_bot 
        lidar_vy = ds / dt


        #Measurement Step
        v_x, v_y = odom
        self.x_velocity.append(v_x)
        self.y_velocity.append(v_y)

        m, c = np.polyfit(np.arange(1, len(self.x_velocity)+1), self.x_velocity, 1)
        velocity_values = np.array(m*np.array(self.x_velocity) + c)
        xk_var = var(velocity_values, np.array(self.x_velocity))

        m, c = np.polyfit(np.arange(1, len(self.y_velocity)+1), self.y_velocity, 1)
        velocity_values = np.array(m*np.array(self.y_velocity) + c)
        yk_var = var(velocity_values, np.array(self.y_velocity))

        
        # Correction Step

        v_x, v_x_var = multiply(v_x, xk_var, lidar_vx, lidarx_k_var)
        v_y, v_y_var = multiply(v_y, yk_var, lidar_vy, lidary_k_var)

        return v_x, v_x_var, v_y, v_y_var

    
    def calc_vel(self, odom, lidar):
        v_x, v_x_var, v_y, v_y_var = self.kalman_filter(odom, lidar)
        
        return v_x, v_x_var, v_y, v_y_var
        
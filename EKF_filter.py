import numpy as np
from time import time
from collections import deque

def predict(pos, variance, movement, movement_variance):
    return (pos + movement, variance + movement_variance)
    
def update(mean, variance, measurement, measurement_variance):
    return multiply(mean, variance, measurement, measurement_variance)
    
def multiply(mu1, var1, mu2, var2):
    mean = (var1*mu2 + var2*mu1) / (var1+var2)
    variance = 1 / (1/var1 + 1/var2)
    return (mean, variance)

class Filter:
    def __init__(self, init_x, init_y):
        #variables at "t-1"
        self.window_size = 5
        self.x_1 = init_x
        self.x_1_cov = 1e-3
        self.y_1 = init_y
        self.y_1_cov = 1e-3
        self.lidar_front_1 = 400 - init_x
        self.lidar_bot_1 = 300 - init_y

        #initialise lidar
        self.lidar_front = deque(maxlen=self.window_size)
        self.lidar_front.append(400 - init_x)
        self.lidar_bot = deque(maxlen=self.window_size)
        self.lidar_bot.append(300 - init_y)

        #initialise odom
        self.x_velocity = deque(maxlen=self.window_size)
        self.x_velocity.append(0.0)
        self.y_velocity = deque(maxlen=self.window_size)
        self.y_velocity.append(0.0)

        #initialise time
        self.time = time()

    def motion_model(self, odom):
        dt = time() - self.time
        self.time = time()
        v_x, v_y = odom
        self.x_velocity.append(v_x)
        self.y_velocity.append(v_y)

        ds_x = dt * v_x 
        pred_x = self.x_1 + ds_x

        ds_y = dt * v_y
        pred_y = self.y_1 + ds_y

        #Handle noise, needs review...
        # small scale linearisation 
        # Find the "linear" velocity, measure variance from the line, metric for noise?

        # numpy covarinace method?
        cov_v_x = np.cov(self.x_velocity, np.arange(1, len(self.x_velocity) + 1, 1))
        cov_v_y = np.cov(self.y_velocity, np.arange(1, len(self.y_velocity) + 1, 1)) 

        #set variables for next iteration
        # self.x_1 = pred_x
        # self.y_1 = pred_y
        #print(f'motion model: x={pred_x}, y={pred_y}')      

        return pred_x, pred_y, cov_v_x, cov_v_y

    def measured_model(self, lidar):
        lidar_front, lidar_bot = lidar
        self.lidar_front.append(lidar_front)
        self.lidar_bot.append(lidar_bot)

        ds_front = self.lidar_front_1 - lidar_front
        measured_x = self.x_1 + ds_front

        ds_bot = self.lidar_bot_1 - lidar_bot
        measured_y = self.y_1 + ds_bot

        #Handle noise, needs review...
        # small scale linearisation 
        # Find the "linear" velocity, measure variance from the line, metric for noise?

        # numpy covarinace method?
        cov_measured_x = np.cov(self.lidar_front, np.arange(1, len(self.lidar_front) + 1, 1))
        cov_measured_y = np.cov(self.lidar_bot, np.arange(1, len(self.lidar_bot) + 1, 1))

        print(f'measured model: x={measured_x}, y={measured_y}')

        return measured_x, measured_y, cov_measured_x, cov_measured_y

    def calc_pose(self, odom, lidar):
        pred_x, pred_y, cov_v_x, cov_v_y = self.motion_model(odom)
        measured_x, measured_y, cov_measured_x, cov_measured_y = self.measured_model(lidar)

        #calc x pose
        self.x_1, self.x_1_cov = multiply(pred_x, cov_v_x, measured_x, cov_measured_x)
    
        #calc y pose
        self.y_1, self.y_1_cov = multiply(pred_y, cov_v_y, measured_y, cov_measured_y)

        print("----------------------------")
        print(f"x_1 = {self.x_1}")
        print(f"y_1 = {self.y_1}")
        self.x_1 = self.x_1[0][0]
        self.y_1 = self.y_1[0][0]

        #self.x_1, self.x_1_cov = measured_x, cov_measured_x
        #self.y_1, self.y_1_cov = measured_y, cov_measured_y

        return self.x_1, self.y_1


import numpy as np
from time import time

class Filter:
    def __init__(self, init_x, init_y):
        self.x_1 = init_x
        self.y_1 = init_y
        self.lidar_1 = 400 - init_x
        self.x_2 = None
        self.x_velocity = None
        self.time = time()

    def est_model(self, v):
        dt = time() - self.time
        self.time = time()
        ds = dt * v 
        est_x = self.x_1 + ds

        #In reality, covariance can be measured through testing
        noise_estimate_v = np.random.normal(loc=0.0, scale=v, size=100)
        cov_v = np.cov(noise_estimate_v)

        return est_x, cov_v

    def measured_model(self, lidar):
        ds = self.lidar_1 - lidar
        measured_x = self.x_1 + ds

        #In reality, covariance can be measured through testing
        noise_estimate_lidar = np.random.normal(loc=0.0, scale=lidar, size=100)
        cov_lidar = np.cov(noise_estimate_lidar)

        return measured_x, cov_lidar

    def predict(self, odom, lidar):
        pass


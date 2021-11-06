import pygame, sys
import numpy as np

class CarSim:
    def __init__(self, x, y, x_max, y_max):
        self.CAR_DIM = [3, 3]
        self.CAR_COLOUR = (200, 0, 0)
        self.X_BOUNDS = [0, x_max]
        self.Y_BOUNDS = [0, y_max]
        self.rect = pygame.Rect(x, y, self.CAR_DIM[0], self.CAR_DIM[1])
        self.cam_center = [300, 200]
        self.rect.x = x
        self.rect.y = y
        self.pose = [0]
        self.momentum = [0, 0]
        self.movement_info = {
            'accel' : 0.3,
            'move_speed' : 3,
            'friction_coeff' : 0.2
        }
        self.keys = {'right': False, 'left': False, 'up': False, 'down': False}
        self.noise_max = 3

    def update(self):
        #movement control
        # if self.move[0]:
        #     self.move_left()
        # elif self.move[1]:
        #     self.move_right()
        # elif abs(self.movement) > 0:
        #     if abs(self.movement < 0.self.noise): self.movement = 0
        #     elif self.movement > 0 : self.movement -= 0.1
        #     else : self.movement += 0.1

        self.update_movement()
        # print(f'x: {self.rect.x}, y: {self.rect.y}')
        # print(f'momentum: {self.momentum[0]}')
        # print(f'keys: {self.keys}')
        # print("------------")

    def update_movement(self):
        # Momentum Control-> needs to be updated for y
        # Momentum Regulation
        if self.momentum[0] > self.movement_info['move_speed']:
            self.momentum[0] = self.movement_info['move_speed']
        if self.momentum[0] < -self.movement_info['move_speed']:
            self.momentum[0] = -self.movement_info['move_speed']
        if self.momentum[1] > self.movement_info['move_speed']:
            self.momentum[1] = self.movement_info['move_speed']
        if self.momentum[1] < -self.movement_info['move_speed']:
            self.momentum[1] = -self.movement_info['move_speed']

        # Speed Down x
        if not self.keys['right'] and not self.keys['left']:
            if self.momentum[0] > 0:
                self.momentum[0] -= self.movement_info['friction_coeff']
            if self.momentum[0] < 0:
                self.momentum[0] += self.movement_info['friction_coeff']
        if not self.keys['right'] and not self.keys['left'] and (
                0.20 > self.momentum[0] > -0.20):
            self.momentum[0] = 0

        # Speed Down y
        if not self.keys['up'] and not self.keys['down']:
            if self.momentum[1] > 0:
                self.momentum[1] -= self.movement_info['friction_coeff']
            if self.momentum[1] < 0:
                self.momentum[1] += self.movement_info['friction_coeff']
        if not self.keys['up'] and not self.keys['down'] and (
                0.20 > self.momentum[1] > -0.20):
            self.momentum[1] = 0

        # Player Movement Control
        if self.keys['right'] and not self.keys['left']:
            self.momentum[0] += self.movement_info['accel']
        if self.keys['left'] and not self.keys['right']:
            self.momentum[0] -= self.movement_info['accel']
        if self.keys['up'] and not self.keys['down']:
            self.momentum[1] -= self.movement_info['accel']
        if self.keys['down'] and not self.keys['up']:
            self.momentum[1] += self.movement_info['accel']
        

        self.rect.x += self.momentum[0]
        self.rect.y += self.momentum[1]

        if self.rect.x + self.CAR_DIM[0] > self.X_BOUNDS[1]: 
            self.rect.x = self.X_BOUNDS[1] - self.CAR_DIM[0]
            self.momentum[0] = 0
        if self.rect.x < self.X_BOUNDS[0]:
            self.rect.x = self.X_BOUNDS[0]
        if self.rect.y + self.CAR_DIM[1] > self.Y_BOUNDS[1]:
            self.rect.y = self.Y_BOUNDS[1] - self.CAR_DIM[1]
            self.momentum[1] = 0
        if self.rect.y < self.Y_BOUNDS[0]:
            self.rect.y = self.Y_BOUNDS[0]
            self.momentum[1] = 0
        #print(f"Movement Speed: {self.momentum[0]}")
        #print("----------------")

    def draw(self, screen, x, y):
        pygame.draw.rect(screen, self.CAR_COLOUR, pygame.Rect(x, y, self.CAR_DIM[0], self.CAR_DIM[1]))
        # screen.blit(player_image, (self.rect.x, self.rect.y))

    def publish_odom(self):
        lat_speed = self.momentum[0] + np.random.normal(loc=0.0, scale=abs(self.noise_func(self.momentum[0])))
        vert_speed = self.momentum[1] + np.random.normal(loc=0.0, scale=abs(self.noise_func(self.momentum[1])))
        return (lat_speed, vert_speed)

    def publish_lidar(self):
        # from back of the car
        x_front = self.X_BOUNDS[1] - self.rect.x #+ self.CAR_DIM[0]
        #x_front += np.random.normal(loc=0.0, scale=self.noise_func(x_front))
        y_bot = self.Y_BOUNDS[1] - self.rect.y + (self.CAR_DIM[1]//2)
        #y_bot += np.random.normal(loc=0.0, scale=self.noise_func(y_bot))
        return x_front, y_bot


    def noise_func(self, speed):
        noise = 0.01*speed**3
        if noise > self.noise_max:
            noise = self.noise_max
        elif noise < -self.noise_max:
            noise = -self.noise_max
        return noise
    
    
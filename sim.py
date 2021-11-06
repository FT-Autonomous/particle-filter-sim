import pygame, sys
from car import CarSim
from loc_kf import KFilter
from vel_kf import Vel_KFilter
from  particle_filter import PFilter
import numpy as np
import os

def noise_func(speed, max=3):
        
        noise = 0.01*speed**3
        if noise > max:
            noise = max
        elif noise < -max:
            noise = -max
        return noise

# General Setup
pygame.init()
clock = pygame.time.Clock()

WIDTH, HEIGHT = 800, 600
WINDOW_SIZE = (WIDTH, HEIGHT)
dis = pygame.display.set_mode(WINDOW_SIZE)
screen = pygame.Surface((400, 300)) # 400, 300
pygame.display.set_caption('Particle Filter Simulator')
SCREEN_MULTI = 3

running = True

barriers = [[200, 100, 20, 150], [200, 250, 150, 20],  [20, 20, 20, 100]]
map_image = np.zeros((400, 300), np.uint8)
for num, barrier in enumerate(barriers):
    map_image[barrier[1]:barrier[1]+barrier[3], barrier[0]:barrier[0]+barrier[2]] = 1

car_sim = CarSim(100, 147, 400, 300)
vel_filter = Vel_KFilter(100, 147)
particle_filter = PFilter(100, 147, map_image, barriers)


if __name__ == '__main__':
    while running:
        # Events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_a:
                    car_sim.keys['left'] = True
                if event.key == pygame.K_d:
                    car_sim.keys['right'] = True
                if event.key == pygame.K_w:
                    car_sim.keys['up'] = True
                if event.key == pygame.K_s:
                    car_sim.keys['down'] = True
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_a:
                    car_sim.keys['left'] = False
                if event.key == pygame.K_d:
                    car_sim.keys['right'] = False
                if event.key == pygame.K_w:
                    car_sim.keys['up'] = False
                if event.key == pygame.K_s:
                    car_sim.keys['down'] = False
            
        screen.fill((80, 80, 200))
        
        # Update Car and publish Odom Results    
        car_sim.update()
        odom_values = car_sim.publish_odom()
        lidar_values = [0, 0]
        lidar_values[0], lidar_values[1] = car_sim.publish_lidar()
        
        for barrier in barriers:
            pygame.draw.rect(screen, (20, 20, 20), pygame.Rect(barrier[0], barrier[1], barrier[2], barrier[3]))
            
            # lidar check
            # Check front
            if car_sim.rect.x <= barrier[0] and barrier[1] <= car_sim.rect.y <= barrier[1] + barrier[3]:
                lidar_values[0] = barrier[0] - car_sim.rect.x
            # Check bot
            if car_sim.rect.y <= barrier[1] and barrier[0] <= car_sim.rect.x <= barrier[0] + barrier[2]:
                lidar_values[1] = barrier[1] - car_sim.rect.y

        lidar_values[0] += np.random.normal(loc=0.0, scale=noise_func(abs(lidar_values[0])))
        lidar_values[1] += np.random.normal(loc=0.0, scale=noise_func(abs(lidar_values[1])))
        
        real_v_x, real_v_y = car_sim.momentum
        pred_v_x, pred_v_x_var, pred_v_y, pred_v_y_var = vel_filter.calc_vel(odom_values, lidar_values)

        pred_x, pred_y, particles, weights = particle_filter.calc_pose((pred_v_x, pred_v_y, pred_v_x_var, pred_v_y_var), lidar_values)

        #Draw particles - Blue
        for num, particle in enumerate(particles):
            pygame.draw.rect(screen, (0, 255*np.power(weights[num], 1/10), 80), pygame.Rect(particle[0], particle[1], car_sim.CAR_DIM[0], car_sim.CAR_DIM[1]))

        #Draws real car pose - red
        car_sim.draw(screen, car_sim.rect.x, car_sim.rect.y)
        #Draws expected pose - Green
        pygame.draw.rect(screen, (0, 255, 80), pygame.Rect(pred_x, pred_y, car_sim.CAR_DIM[0], car_sim.CAR_DIM[1]))
        

        surf = pygame.transform.scale(screen, WINDOW_SIZE)
        dis.blit(surf, (0, 0))
        pygame.display.flip()
        clock.tick(60)
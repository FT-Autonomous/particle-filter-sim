# Particle Filter Sim
Simple Pygame simulator for demonstrating how a particle filter works. Only 2D lidar data is fed to the car in order to make the simulation simple to use, just the x ( distance from the car to the right side of the screen ) and y ( distance to the bottom of the screen ). 

## Requirements
- [pygame](https://www.pygame.org/news)
- [numpy](https://numpy.org)

## How to Run the Simulator 
To run the simulator, just run the following command in your terminal  :
```
python sim.py
```

## Controls
You can control the movement of the car with your W A S D keys.
The red box drawn on the screen is the real postion of the car. The bright green box drawn on the car is the postion of the car guessed by the particle filter. You'll also see the particles as the car moves, these particles are coloured according to their weights. 
The particles are given a bluer colour if the particle filter is less confident in that guess ( lower weight ) and a greener colour if the particle filter is more confident in that guess ( higher weight ).

## Simulator Variables 
The frame rate of the simulator can be adjusted in the `sim.py` file by changing the `clock.tick(frame_rate)` line.
Barriers are drawn onto the screen in the form of rectangles with `pygame.Rect()`. More barriers can be added by changing the `barriers` array. Add these barriers as an array of length four with the following format : 
```
[x_pos, y_pos, x_length, y_length]
```
Note here that the screen has dimensions of `(400, 300)` and that in pygame down is the positive direction of the y-axis.

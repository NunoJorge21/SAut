import math
import pygame
import time
from ROBOT import Graphics, Robot, Lidar#, Ultrasonic

MAP_DIMENSIONS = (600, 1200)

# environment graphics
gfx = Graphics(MAP_DIMENSIONS, 'DDR.png', 'ObstacleMap.png')

# robot
start = (200, 200)
robot = Robot(start, 0.01*3779.52)

bearing=360
# sensor
sensor_range = 250, math.radians(bearing/2) #250,rad(40)
#ultra_sonic = Ultrasonic(sensor_range, gfx.map)
lidar = Lidar(sensor_range, gfx.map)

dt = 0
last_time = pygame.time.get_ticks()

running = True

# simulation loop

while running:
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        keys = pygame.key.get_pressed() 
        if keys[pygame.K_w]:
            robot.move_forward()
            robot.kinematics(dt)
            print(robot.x)
            print(robot.y)
            print(robot.heading)
        if keys[pygame.K_s]:
            robot.move_backward()
            robot.kinematics(dt)
            print(robot.x)
            print(robot.y)
            print(robot.heading)
        if keys[pygame.K_a]:
            robot.move_left()
            robot.kinematics(dt)
            print(robot.x)
            print(robot.y)
            print(robot.heading)
        if keys[pygame.K_d]:
            robot.move_right()
            robot.kinematics(dt)
            print(robot.x)
            print(robot.y)
            print(robot.heading)
    
    dt = (pygame.time.get_ticks() - last_time)/1000
    last_time = pygame.time.get_ticks()
    
    gfx.map.blit(gfx.map_img, (0, 0))

    #robot.kinematics(dt)
    gfx.draw_robot(robot.x, robot.y, robot.heading)
    #point_cloud = ultra_sonic.sense_obstacles(robot.x, robot.y, robot.heading)
    point_cloud = lidar.sense_obstacles(robot.x, robot.y, robot.heading)
    #robot.avoid_obstacles(point_cloud, dt)
    gfx.draw_sensor_data(point_cloud)

    pygame.display.update()

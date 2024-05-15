import math
import pygame
import time
from tkinter import *

from ROBOT import Graphics, Robot, Lidar#, Ultrasonic

MAP_DIMENSIONS = (600, 1200)

'''
Updates map window
Arguments:
    map - map window (of class Canvas)
    pixel_position - pixel to change ((x,y) coordinates)
    color - new color of the pixel (string)
'''
def update_map(map, pixel_position, color):
    map.create_rectangle(pixel_position[0], pixel_position[1], pixel_position[0] + 1, pixel_position[1] + 1, fill=color, outline="")


map_matrix = [[0.5 for i in range(1200)] for j in range(600)] # 600x1200 matrix of occupancy cells (0.5 -> maximum uncertainty)


# window for occupancy grid map
window = Tk()
window.title("Occupancy Grid Map")
window.geometry("1200x600")

map_window = Canvas(window, width=1200, height=600, bg="#888A85")
map_window.pack()


# environment graphics
gfx = Graphics(MAP_DIMENSIONS, 'DDR.png', 'ObstacleMap.png')

# robot
start = (200, 200)
robot = Robot(start, 0.01*3779.52)

# sensor
angular_range=360
sensor_range = 250, math.radians(angular_range/2) #250,rad(40)
#ultra_sonic = Ultrasonic(sensor_range, gfx.map)
lidar = Lidar(sensor_range, gfx.map)

dt = 0
last_time = pygame.time.get_ticks()

running = True


# simulation loop

while running:
    window.update_idletasks()
    window.update()
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        keys = pygame.key.get_pressed() 
        if keys[pygame.K_w]:
            robot.move_forward()
            robot.kinematics(dt)
        if keys[pygame.K_s]:
            robot.move_backward()
            robot.kinematics(dt)
        if keys[pygame.K_a]:
            robot.move_left()
            robot.kinematics(dt)
        if keys[pygame.K_d]:
            robot.move_right()
            robot.kinematics(dt)

    
    dt = (pygame.time.get_ticks() - last_time)/1000
    last_time = pygame.time.get_ticks()
    
    gfx.map.blit(gfx.map_img, (0, 0))

    #robot.kinematics(dt)
    
    gfx.draw_robot(robot.x, robot.y, robot.heading)

    #point_cloud = ultra_sonic.sense_obstacles(robot.x, robot.y, robot.heading)
    point_cloud = lidar.sense_obstacles(robot.x, robot.y, robot.heading)
    for point in point_cloud:
        update_map(map_window, point, "black")

    #robot.avoid_obstacles(point_cloud, dt)
    
    gfx.draw_sensor_data(point_cloud)

    # might have to specify condition to tell it's the first time 
    #update_map()

    pygame.display.update()

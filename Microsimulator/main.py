import math
import pygame
import time
from tkinter import *

from ROBOT import Graphics, Robot, Lidar#, Ultrasonic

MAP_DIMENSIONS = (600, 1200)


'''
Draw map in map_window
Arguments:
    current_l_i - current representation of map with log odds (matrix)
    map - window where the map is being drawn (object of class Canvas)
'''
def draw_map(current_l_i, map):
    n_rows = len(current_l_i)
    n_cols = len(current_l_i[0])

    for row in range(n_rows):
        for col in range(n_cols):
            if current_l_i[row][col] > 0:
                # occupied cell
                map.create_rectangle(row, col, row + 1, col + 1, fill="black", outline="")
            elif current_l_i[row][col] < 0:
                # free cell
                map.create_rectangle(row, col, row + 1, col + 1, fill="white", outline="")


'''
Inverse Sensor Model
Arguments:
    map - map matrix
    current_state - current location of the robot (not pose; we may need to change this if sensor's angle range smaller than 2*pi)
    current_observations - location of obstacles in our current perceptual field
    sensor_range - range of the sensor in pixels (allows to check if a particular cell is in our current perceptual field)
'''
def inverse_sensor_model(map, current_state, current_observations):
    return 0



'''
Occupancy Grid Mapping Algorithm
Arguments:
    preavious_l_i - preavious representation of map with log odds
    current_state - current location of the robot (not pose; we may need to change this to consider angle range smaller than 2*pi)
    current_observations - location of obstacles in our current perceptual field
    map - map matrix
    sensor_range - ditance (in pixels) and angular (in radians) range of the sensor
'''
def occupancy_grid_mapping(preavious_l_i, current_state, current_observations, map, sensor_range):
    n_rows = len(map)
    n_cols = len(map[0])

    for row in range(n_rows):
        for column in range(n_cols):
            if math.dist((current_state[0], current_state[1]), (row, column)) <= sensor_range[0]:
                # cell in perceptual field of observation
                current_l_i = preavious_l_i # + inverse_sensor_model(map, current_state, current_observations)         # l0 = 0, since prior = 0.5
            else:
                current_l_i = preavious_l_i
    
    return current_l_i


map_matrix = [[0.5 for i in range(1200)] for j in range(600)] # 600x1200 matrix of occupancy cells (0.5 -> maximum uncertainty)
l_i = [[0.0 for i in range(1200)] for j in range(600)] # log odds representation of occupancy


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

    # update and draw map
    l_i = occupancy_grid_mapping(l_i, [robot.x, robot.y], point_cloud, map_matrix, sensor_range)
    draw_map(l_i, map_window)

    #robot.avoid_obstacles(point_cloud, dt)
    
    gfx.draw_sensor_data(point_cloud)

    pygame.display.update()

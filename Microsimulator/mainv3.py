import math
import pygame
import time
from tkinter import *
import numpy as np
from ROBOTv3 import Graphics, Robot, Lidar#, Ultrasonic
import random

# Constants for log-odds update
LOG_ODDS_FREE = -1.0
LOG_ODDS_OCCUPIED = 1.0

def bresenham(begin,end):
    x0 = int(begin[0])
    y0 = int(begin[1])
    x1 = int(end[0])
    y1 = int(end[1])

    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    points.append((int(x1), int(y1)))

    return points[:-1] # does not return last element


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

def add_noise(distance, stddev=1.0):
    return distance + random.gauss(0, stddev)

'''
Inverse Sensor Model
Arguments:
    cell_i - position of the current cell of the map (array with 2 elements)
    current_state - current location of the robot (not pose; we may need to change this if sensor's angle range smaller than 2*pi)
    current_observations - location of obstacles in our current perceptual field
    sensor_range - range of the sensor in pixels (allows to check if a particular cell is in our current perceptual field)
'''
#def inverse_sensor_model(cell_i, previous_l_i, current_state, current_observations, alpha=1, beta=1):

    #VER DO BETA
    #r = math.sqrt((cell_i[0] - current_state[0])**2 + (cell_i[1] - current_state[1])**2)   
    #phi=math.atan2(cell_i[1]-current_state[1],cell_i[0]-current_state[0]) - current_state[2]
    # FALTA k

    #a = 
    #b = math.dist((current_observations[k][0], current_observations[k][1]), (current_state[0], current_state[1])) + alpha/2

    #current_obs[360] = min(max range, range of obstacle)

    #if r > min(max(current_observations), b) or :

    #current_l_i = previous_l_i

    #for obstacle in current_observations:
    #    line_points = bresenham(current_state, obstacle)
    #    for point in line_points:
    #        current_l_i[point[0]][point[1]] = -1
    #    
    #    current_l_i[obstacle[0]][obstacle[1]] = 1
    
    #return current_l_i

def inverse_sensor_model(cell_i, previous_l_i, current_state, current_observations, alpha=1, beta=1):
    x, y = cell_i
    x_r, y_r, theta_r = current_state

    r = math.sqrt((x - x_r)**2 + (y - y_r)**2)
    phi = math.atan2(y - y_r, x - x_r) - theta_r

    if r > sensor_range[0] or abs(phi) > sensor_range[1]:
        return previous_l_i[x][y]

    for obs in current_observations:
        r_obs = math.sqrt((obs[0] - x_r)**2 + (obs[1] - y_r)**2)
        if r < r_obs:
            return previous_l_i[x][y] + LOG_ODDS_FREE
        elif r == r_obs:
            return previous_l_i[x][y] + LOG_ODDS_OCCUPIED

    return previous_l_i[x][y]

'''
Occupancy Grid Mapping Algorithm
Arguments:
    previous_l_i - previous representation of map with log odds
    current_state - current locationobstacles.append([x, y, 1])
                        break of the robot (not pose; we may need to change this to consider angle range smaller than 2*pi)
    current_observations - location of obstacles in our current perceptual field
    map - map matrix
    sensor_range - ditance (in pixels) and angular (in radi    line_points = bresenham(current_state,[x_obs, y_obs])
        
        for point in line_points:
            current_l_i[point[0]][point[1]] = -1ans) range of the sensor
'''
def occupancy_grid_mapping(previous_l_i, current_state, current_observations, map, sensor_range):
    
    '''
    for row in range(n_rows):
        for column in range(n_cols):
            if math.dist((current_state[0], current_state[1]), (row, column)) <= sensor_range[0]:
                # cell in perceptual field of observation
                current_l_i = previous_l_i + inverse_sensor_model([row, column], previous_l_i, current_state, current_observations)         # l0 = 0, since prior = 0.5
            else:
                current_l_i = previous_l_i
    '''

    current_l_i = previous_l_i

    for obstacle in current_observations:
        line_points = bresenham(current_state, obstacle)
        for point in line_points:
            current_l_i[point[0]][point[1]] = -1
        
        if obstacle[2] == 1:
            current_l_i[obstacle[0]][obstacle[1]] = 1
        elif obstacle[2] == 0:
            current_l_i[obstacle[0]][obstacle[1]] = -1
    current_l_i = [row[:] for row in previous_l_i]

    #for obs in current_observations:
    #    line_points = bresenham((current_state[0], current_state[1]), (obs[0], obs[1]))
    #    for point in line_points:
    #        current_l_i[point[1]][point[0]] = inverse_sensor_model(point, previous_l_i, current_state, current_observations)
    #    current_l_i[obs[1]][obs[0]] = inverse_sensor_model((obs[0], obs[1]), previous_l_i, current_state, current_observations)
    
    return current_l_i


#map_matrix = [[0.5 for i in range(1200)] for j in range(600)] # 600x1200 matrix of occupancy cells (0.5 -> maximum uncertainty)
#l_i = [[0.0 for i in range(1200)] for j in range(600)] # log odds representation of occupancy

map_matrix = [[0.5 for i in range(00)] for j in range(600)] # 600x1200 matrix of occupancy cells (0.5 -> maximum uncertainty)
l_i = [[0.0 for i in range(600)] for j in range(1200)] # log odds representation of occupancy


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
angular_range = 360
sensor_range = 250, math.radians(angular_range/2) #250,rad(40)
#ultra_sonic = Ultrasonic(sensor_range, gfx.map)
lidar = Lidar(sensor_range, gfx.map)

dt = 0
last_time = pygame.time.get_ticks()

running = True

a = 0
n = 0
# simulation loop

while running:
    window.update_idletasks()
    window.update()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        '''
        keys = pygame.key.get_pressed() 
        if keys[pygame.K_w]:
            robot.move_forward()
            robot.kinematics(dt)
            a=1
        if keys[pygame.K_s]:
            robot.move_backward()
            robot.kinematics(dt)
            a=1
        if keys[pygame.K_a]:
            robot.move_left()
            robot.kinematics(dt)
            a=1
        if keys[pygame.K_d]:
            robot.move_right()
            robot.kinematics(dt)
            a=1
            '''
        
    
    dt = (pygame.time.get_ticks() - last_time)/1000
    last_time = pygame.time.get_ticks()
    
    gfx.map.blit(gfx.map_img, (0, 0))

    gfx.draw_robot(robot.x, robot.y, robot.heading)

    point_cloud = lidar.sense_obstacles(robot.x, robot.y, robot.heading)
    
    noisy_point_cloud = [(add_noise(point[0]), add_noise(point[1]), point[2]) for point in point_cloud]
    
    if n <= 25:
        #robot.kinematics(dt)
        #robot.avoid_obstacles(point_cloud, dt)
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


    #if a==1:
    if n > 10:
        l_i = occupancy_grid_mapping(l_i, [robot.x, robot.y], point_cloud, map_matrix, sensor_range)
        draw_map(l_i, map_window)
        n = 0
    #    a=0

    #robot.avoid_obstacles(point_cloud, dt)
    
    gfx.draw_sensor_data(noisy_point_cloud)
    
    n = n+1

    pygame.display.update()





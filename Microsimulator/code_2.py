import math
import pygame
import time
from tkinter import *
import numpy as np
from ROBOTv3 import Graphics, Robot, Lidar, Cell
import random
import copy
from scipy import stats


MAP_DIMENSIONS = (600, 1200)

LIDAR_ERROR = 0.035*3779.53 # LiDAR's precision is 3.5% of the measured distance 
BETA = math.radians(0.1)


def add_noise(distance, stddev=1.0):
    #return distance + random.gauss(0, stddev) # adds zero-meaned noise
    return distance + stats.truncnorm.rvs(-5/stddev, 5/stddev, loc=0, scale=stddev, size=1)[0]


'''
Bresenham's line algorithm; chooses pixels to that connect two points, creating a line

Arguments:
    begin - beginning point
    end - ending point
'''
def bresenham(begin, end):
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
    points.append((int(x1), int(y1)))

    return points[:-1]


'''
Draw map in map_window
Arguments:
    current_l_i - current representation of map with log odds (matrix)
    map - map matrix
    map_window - window where the map is being drawn (object of class Canvas)
'''
def draw_map(current_l_i, map, map_window):
    map_window.delete("all")

    for index, cell in enumerate(map):
        if current_l_i[index] > 0:
            map_window.create_rectangle(cell.x, cell.y, cell.x + 1, cell.y + 1, fill="black", outline="")
        elif current_l_i[index] < 0:
            map_window.create_rectangle(cell.x, cell.y, cell.x + 1, cell.y + 1, fill="white", outline="")


'''
def l0(stddev):
    p = random.gauss(0.5, stddev)
    return math.log(p/(1-p))
'''

def l_occ(stddev):
    p = 1 - stats.truncnorm.rvs(0, 1/stddev, loc=0, scale=stddev, size=1)[0]
    return math.log(p/(1-p))

def l_free(stddev):
    p = 0 + stats.truncnorm.rvs(0, 1/stddev, loc=0, scale=stddev, size=1)[0]
    return math.log(p/(1-p))


'''
Inverse Sensor Model
Arguments:
    cell_i - position of the current cell of the map (array with 2 elements)
    current_state - current location of the robot (not pose; we may need to change this if sensor's angle range smaller than 2*pi)
    current_observations - location of obstacles in our current perceptual field
    sensor_range - range of the sensor in pixels (allows to check if a particular cell is in our current perceptual field)
'''
def inverse_sensor_model(cell_i, current_state, current_observations, sensor_range):
    ###############################################################
    # teremos de passar distâncias para metros, converter para píxeis...

    x = cell_i.x 
    y = cell_i.y
    x_r, y_r, theta_r = current_state

    r = math.sqrt((x - x_r)**2 + (y - y_r)**2)
    phi = math.atan2(y - y_r, x - x_r) - theta_r

    theta_list = np.array([element[1] for element in current_observations])
    differences = np.abs(phi - theta_list)
    k = np.argmin(differences)

    ##############################################
    # tratamos do alpha depois
    if r > sensor_range[0]:
        #return l0(LIDAR_ERROR)
        return 0.0
    
    if current_observations[k][0] < sensor_range[0] or differences[k] > BETA/2:
        if r == 0:
            return math.inf
        
        return l_occ(LIDAR_ERROR*r)
    
    if r <= current_observations[k][0]:
        if r == 0:
            return -math.inf
        return l_free(LIDAR_ERROR*r)

    return 0.0



'''
Occupancy Grid Mapping Algorithm
Arguments:
    previous_l_i - previous representation of map with log odds
    current_state - robot's current pose
    current_observations - location of obstacles in our current perceptual field
    map - map matrix (array of elements of clas Cell)
    sensor_range - ditance (in pixels) and angular (in radi    line_points = bresenham(current_state,[x_obs, y_obs])
'''
def occupancy_grid_mapping(previous_l_i, current_state, current_observations, map, sensor_range):
    current_l_i = copy.deepcopy(previous_l_i)

    for index, cell in enumerate(map):
        if cell.distance_to_state(current_state[:-1]) <= sensor_range[0]:
            # current cell is in sensor range
            current_l_i[index] += inverse_sensor_model(cell, current_state, current_observations, sensor_range)

    return current_l_i            




#############################################
# gray scale
#####################################

# Initialize map and log-odds matrix
map_matrix = []
l_i = []

for i in range(MAP_DIMENSIONS[0]):
    for j in range(MAP_DIMENSIONS[1]):
        cell = Cell(i, j)
        map_matrix.append(cell)
        l_i.append(0.0)

# GUI setup
window = Tk()
window.title("Occupancy Grid Map")
window.geometry("1200x600")
map_window = Canvas(window, width=1200, height=600, bg="#888A85")
map_window.pack()

# Environment graphics
gfx = Graphics(MAP_DIMENSIONS, 'DDR.png', 'ObstacleMap.png')

# Robot setup
start = (200, 200)
robot = Robot(start, 0.01 * 3779.52)

# Sensor setup
angular_range = 360
sensor_range = 250, math.radians(angular_range / 2)
lidar = Lidar(sensor_range, gfx.map)

dt = 0
last_time = pygame.time.get_ticks()

running = True
n = 0

while running:
    window.update_idletasks()
    window.update()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    dt = (pygame.time.get_ticks() - last_time) / 1000
    last_time = pygame.time.get_ticks()

    gfx.map.blit(gfx.map_img, (0, 0))
    gfx.draw_robot(robot.x, robot.y, robot.heading)

    point_cloud = lidar.sense_obstacles(robot.x, robot.y, robot.heading) # returns distance and angle of obstacle
    
    # Add noise to Lidar measurements
    noisy_point_cloud = [(add_noise(point[0], LIDAR_ERROR*point[0]), point[1]) for point in point_cloud]

    if n <= 60:
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

    if n > 60:
        l_i = occupancy_grid_mapping(l_i, [robot.x, robot.y, robot.heading], noisy_point_cloud, map_matrix, sensor_range)
        draw_map(l_i, map_matrix, map_window)
        n = 0

    gfx.draw_sensor_data(noisy_point_cloud, [robot.x, robot.y, robot.heading])
    
    n += 1
    pygame.display.update()
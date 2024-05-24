import math
import pygame
import time
from tkinter import *
import numpy as np
from ROBOTv4 import Graphics, Robot, Lidar, Cell
import copy
from scipy import stats


MAP_DIMENSIONS = (600, 1200)
LIDAR_ERROR = 0.035*3779.53 # LiDAR's precision is 3.5% of the measured distance


def add_noise(distance, stddev=1.0):
    #return distance + random.gauss(0, stddev) # adds zero-meaned noise
    return distance + stats.truncnorm.rvs(-5/stddev, 5/stddev, loc=0, scale=stddev, size=1)[0] 


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
    #points.append((int(x1), int(y1)))

    return points # does not return last element


def l_occ(stddev):
    p = 1 - stats.truncnorm.rvs(0, 1/stddev, loc=0, scale=stddev, size=1)[0]
    #return math.log(p/(1-p))
    return 1

def l_free(stddev):
    p = 0 + stats.truncnorm.rvs(0, 1/stddev, loc=0, scale=stddev, size=1)[0]
    #return math.log(p/(1-p))
    return -1


'''
Draw map in map_window
Arguments:
    current_l_i - current representation of map with log odds (matrix)
    map - map_matrix
    map_window - window where the map is being drawn (object of class Canvas)
'''
def draw_map(current_l_i, map, map_window):
    for index, cell in enumerate(map):
        if current_l_i[index] > 0:
            map_window.create_rectangle(cell.x, cell.y, cell.x + 1, cell.y + 1, fill="black", outline="")
        elif current_l_i[index] < 0:
            map_window.create_rectangle(cell.x, cell.y, cell.x + 1, cell.y + 1, fill="white", outline="")


'''
Inverse Sensor Model
Arguments:
    pixel - coordinates of the pixel under consideration
    occupied_pixels - list of pixels that are (supposedly) occupied
    free_pixels - list of pixels that are (supposedly) free
'''
def inverse_sensor_model(pixel, occupied_pixels, free_pixels):
    if pixel in occupied_pixels:
        return l_occ(LIDAR_ERROR)
    
    if pixel in free_pixels:
        return l_free(LIDAR_ERROR)
    
    return 0.0


def distance2pixel(pose, observation):
    r = observation[0]
    theta = observation[1]
    #phi = theta + pose[2]
    phi = observation[1]

    x = int(pose[0] + r * math.cos(phi))
    #y = int(pose[1] + r * math.sin(phi))
    y = int(pose[1] - r * math.sin(phi))

    return [x, y]


'''
Occupancy Grid Mapping Algorithm
Arguments:
    previous_l_i - previous representation of map with log odds
    current_state - robot's current pose
    current_observations - location of obstacles in our current perceptual field
    map - map matrix (array of elements of clas Cell)
    sensor_range - ditance (in pixels) and angular range (in radians)
'''
def occupancy_grid_mapping(previous_l_i, current_state, current_observations, map, sensor_range):
    current_l_i = copy.deepcopy(previous_l_i)
    new_occupied_pixels = []
    new_free_pixels = []

    for observation in current_observations:
        ending_pixel = distance2pixel(current_state, observation)
        new_occupied_pixels.append(ending_pixel)
        line = bresenham(current_state[:-1], ending_pixel) # line does not include ending pixel
        new_free_pixels.append(line)

    for index, cell in enumerate(map):
        distance = cell.distance_to_state(current_state[:-1]) 
        if distance <= sensor_range[0]:
            # current cell is in sensor range
            current_l_i[index] += inverse_sensor_model([cell.x, cell.y], new_occupied_pixels, new_free_pixels)

    return current_l_i   




####################################
# --------------main-------------- #

# Initialize map and log-odds matrix
map_matrix = []
l_i = []

for i in range(MAP_DIMENSIONS[0]):
    for j in range(MAP_DIMENSIONS[1]):
        cell = Cell(i, j)
        map_matrix.append(cell)
        l_i.append(0.0)


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
lidar = Lidar(sensor_range, gfx.map)

dt = 0
last_time = pygame.time.get_ticks()

running = True

n = 0

# simulation loop
while running:
    window.update_idletasks()
    window.update()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
   
    dt = (pygame.time.get_ticks() - last_time)/1000
    last_time = pygame.time.get_ticks()
    
    gfx.map.blit(gfx.map_img, (0, 0))

    gfx.draw_robot(robot.x, robot.y, robot.heading)

    point_cloud = lidar.sense_obstacles(robot.x, robot.y, robot.heading)
    # Add noise to Lidar measurements
    noisy_point_cloud = [(add_noise(point[0], LIDAR_ERROR*point[0]), point[1]) for point in point_cloud]
    
    if n <= 25:
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


    if n > 25:
        l_i = occupancy_grid_mapping(l_i, [robot.x, robot.y, robot.heading], noisy_point_cloud, map_matrix, sensor_range)
        draw_map(l_i, map_matrix, map_window)
        n = 0
    
    gfx.draw_sensor_data(noisy_point_cloud, [robot.x, robot.y, robot.heading])
    
    n = n+1

    pygame.display.update()
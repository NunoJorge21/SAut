import math
import pygame
import time
from tkinter import *
import numpy as np
from ROBOTv3 import Graphics, Robot, Lidar
import random

# Constants for log-odds update
LOG_ODDS_FREE = -1.0
LOG_ODDS_OCCUPIED = 1.0
LOG_ODDS_PRIOR = 0.0

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

MAP_DIMENSIONS = (600, 1200)

def draw_map(current_l_i, map):
    n_rows = len(current_l_i)
    n_cols = len(current_l_i[0])

    map.delete("all")

    for row in range(n_rows):
        for col in range(n_cols):
            if current_l_i[row][col] > 0:
                map.create_rectangle(col, row, col + 1, row + 1, fill="black", outline="")
            elif current_l_i[row][col] < 0:
                map.create_rectangle(col, row, col + 1, row + 1, fill="white", outline="")

def add_noise(distance, stddev=1.0):
    return distance + random.gauss(0, stddev)

def inverse_sensor_model(cell_i, current_state, current_observations, sensor_range):
    x, y = cell_i
    x_r, y_r, theta_r = current_state

    r = math.sqrt((x - x_r)**2 + (y - y_r)**2)
    phi = math.atan2(y - y_r, x - x_r) - theta_r

    if r > sensor_range[0] or abs(phi) > sensor_range[1]:
        return LOG_ODDS_PRIOR

    for obs in current_observations:
        r_obs = math.sqrt((obs[0] - x_r)**2 + (obs[1] - y_r)**2)
        if r < r_obs:
            return LOG_ODDS_FREE
        elif r == r_obs:
            return LOG_ODDS_OCCUPIED

    return LOG_ODDS_PRIOR

def occupancy_grid_mapping(previous_l_i, current_state, current_observations, sensor_range):
    current_l_i = [row[:] for row in previous_l_i]

    for obs in current_observations:
        line_points = bresenham((current_state[0], current_state[1]), (obs[0], obs[1]))
        for point in line_points:
            x, y = point
            if 0 <= x < len(current_l_i[0]) and 0 <= y < len(current_l_i):
                current_l_i[y][x] += inverse_sensor_model(point, current_state, current_observations, sensor_range)
        obs_x, obs_y = int(obs[0]), int(obs[1])
        if 0 <= obs_x < len(current_l_i[0]) and 0 <= obs_y < len(current_l_i):
            current_l_i[obs_y][obs_x] += inverse_sensor_model((obs_x, obs_y), current_state, current_observations, sensor_range)

    return current_l_i

# Initialize map and log-odds matrix
map_matrix = [[0.5 for _ in range(1200)] for _ in range(600)]
l_i = [[0.0 for _ in range(1200)] for _ in range(600)]

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

    point_cloud = lidar.sense_obstacles(robot.x, robot.y, robot.heading)
    
    # Add noise to Lidar measurements
    noisy_point_cloud = [(add_noise(point[0]), add_noise(point[1]), point[2]) for point in point_cloud]

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
        l_i = occupancy_grid_mapping(l_i, [robot.x, robot.y, robot.heading], noisy_point_cloud, sensor_range)
        draw_map(l_i, map_window)
        n = 0

    gfx.draw_sensor_data(noisy_point_cloud)
    
    n += 1
    pygame.display.update()

import math
import pygame
import time
from tkinter import *

from ROBOT import Graphics, Robot, Lidar#, Ultrasonic

 
def p_normal(x, u, sigma2): 
    return math.exp(-(x-u)**2/(2*sigma2)) / math.sqrt(2*math.pi*sigma2)

def lo(p):
    return math.log(p/(1.0-p))

def plo(l):
    return 1 - 1/(1+math.exp(l))

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
    previous_l_i - previous representation of map with log odds
    current_state - current location of the robot (not pose; we may need to change this to consider angle range smaller than 2*pi)
    current_observations - location of obstacles in our current perceptual field
    map - map matrix
    sensor_range - ditance (in pixels) and angular (in radians) range of the sensor
'''
def occupancy_grid_mapping(previous_l_i, current_state, current_observations, map, sensor_range):
    n_rows = len(map)
    n_cols = len(map[0])

    for row in range(n_rows):
        for column in range(n_cols):
            if math.dist((current_state[0], current_state[1]), (row, column)) <= sensor_range[0]:
                # cell in perceptual field of observation
                current_l_i = previous_l_i # + inverse_sensor_model(map, current_state, current_observations)         # l0 = 0, since prior = 0.5
            else:
                current_l_i = previous_l_i
    
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

a=0
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

    
    dt = (pygame.time.get_ticks() - last_time)/1000
    last_time = pygame.time.get_ticks()
    
    gfx.map.blit(gfx.map_img, (0, 0))

    #robot.kinematics(dt)
    
    gfx.draw_robot(robot.x, robot.y, robot.heading)

    #point_cloud = ultra_sonic.sense_obstacles(robot.x, robot.y, robot.heading)
    point_cloud = lidar.sense_obstacles(robot.x, robot.y, robot.heading)

    if a==1:
        l_i = occupancy_grid_mapping(l_i, [robot.x, robot.y], point_cloud, map_matrix, sensor_range)
        draw_map(l_i, map_window)
        a=0

    #robot.avoid_obstacles(point_cloud, dt)
    
    gfx.draw_sensor_data(point_cloud)

    pygame.display.update()

'''
    import numpy as np

def inverse_sensor_model(occupancy_grid, robot_pos, observations):
    # Calculate the endpoints of the rays
    ray_endpoints = calculate_ray_endpoints(robot_pos, observations)

    # Calculate the probabilities of occupancy for each cell along the rays
    for i, ray in enumerate(ray_endpoints):
        probs = []
        for x, y in ray:
            # Calculate the likelihood of the current observation, given that the cell is occupied
            likelihood = calculate_likelihood(observations[i], x, y)

            # Calculate the prior probability of the cell being occupied
            prior = occupancy_grid[x][y]

            # Calculate the posterior probability of the cell being occupied, given the current observation and the prior probability
            posterior = (likelihood * prior) / (likelihood * prior + (1 - likelihood) * (1 - prior))

            # Add the probability to the list
            probs.append(posterior)

        # Update the occupancy grid with the new probabilities of occupancy
        for x, y, prob in zip(*np.array(ray).T, probs):
            occupancy_grid[x][y] = prob

    return occupancy_grid

def calculate_ray_endpoints(robot_pos, observations):
    # Calculate the endpoints of the rays
    ray_endpoints = []
    for observation in observations:
        # Calculate the angle of the ray
        angle = np.arctan2(observation[1] - robot_pos[1], observation[0] - robot_pos[0])

        # Calculate the endpoints of the ray
        endpoint = (robot_pos[0] + observation[2] * np.cos(angle), robot_pos[1] + observation[2] * np.sin(angle))

        # Add the endpoints to the list
        ray_endpoints.append

        import math

def calculate_likelihood(observation, x, y):
    # Calculate the distance between the current position and the observed position
    dist = math.sqrt((x - observation[0])**2 + (y - observation[1])**2)

    # Calculate the likelihood of the observation, given the current position
    likelihood = math.exp(-(dist - observation[2])**2 / (2 * observation[3]**2))

    return likelihood
'''
#observation[0]: The x-coordinate of the observed position.
#observation[1]: The y-coordinate of the observed position.
#observation[2]: The true distance to the obstacle.
#observation[3]: The standard deviation of the Gaussian distribution.

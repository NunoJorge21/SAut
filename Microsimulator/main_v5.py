import math
import pygame
from tkinter import *
from ROBOTv4 import Graphics, Robot, Lidar, Cell
from scipy import stats


MAP_DIMENSIONS = (300, 450)
LIDAR_ERROR = 0.035*3779.53 # LiDAR's precision is 3.5% of the measured distance
SENSOR_RANGE = 250, math.pi

# Probability threshold for identifying occupied (upper) and free (lower) cells
##################################################################
# -------> EXPERIMENTAR VÁRIOS VALORES de p_upper e p_lower!!!!!!!
# Não alterar as outras coisas!
##################################################################
p_upper = 0.55 
p_lower = 0.45
UPPER_THRESHOLD = math.log(p_upper/(1-p_upper))
LOWER_THRESHOLD = math.log(p_lower/(1-p_lower))

global p_occ
global p_free

p_occ = 0.68 # P(u-s < X < u+s) = 0.68, for X ~ N(u,s)
p_free = 1-p_occ

global map_window


def add_noise(value, stddev=1.0):
    return value + stats.truncnorm.rvs(-5/stddev, 5/stddev, loc=0, scale=stddev, size=1)[0] 


def bresenham(begin, end, free_cells):
    x0 = int(begin[0])
    y0 = int(begin[1])
    x1 = int(end[0])
    y1 = int(end[1])

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            free_cells.append([x, y])
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            free_cells.append([x, y])
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy

    # does not return last element
    return free_cells


'''
Arguments:
    d - distance read by the sensor (in the same unit as SENSOR_RANGE[0])
'''
def l_occ(d):
    w = (SENSOR_RANGE[0] - 0.035*d)/SENSOR_RANGE[0]
    return math.log(w*p_occ/(1-p_occ))

'''
Arguments:
    d - distance read by the sensor (in the same unit as SENSOR_RANGE[0])
'''
def l_free(d):
    w = (SENSOR_RANGE[0] - 0.035*d)/SENSOR_RANGE[0]
    return math.log(w*p_free/(1-p_free))


'''
Draw map in map_window
Arguments:
    current_l_i - current representation of map with log odds (matrix)
    map - map_matrix
'''
def draw_map(current_l_i, map):
    for index, cell in enumerate(map):
        if current_l_i[index] > UPPER_THRESHOLD:
            # occupied cell
            map_window.create_rectangle(cell.x, cell.y, cell.x + 1, cell.y + 1, fill="black", outline="")
        elif current_l_i[index] < LOWER_THRESHOLD:
            # free cell
            map_window.create_rectangle(cell.x, cell.y, cell.x + 1, cell.y + 1, fill="white", outline="")



'''
Inverse Sensor Model
Arguments:
    pixel - coordinates of the pixel under consideration
    occupied_pixels - list of pixels that are (supposedly) occupied
    free_pixels - list of pixels that are (supposedly) free
    distance - distance between the robot's current location and pixel 
'''
def inverse_sensor_model(pixel, occupied_pixels, free_pixels, distance):
    if pixel in occupied_pixels:
        return l_occ(distance)
    
    if pixel in free_pixels:
        return l_free(distance)
    
    return 0.0


def distance2pixel(pose, observation):
    r = observation[0]
    theta = observation[1]

    x = int(pose[0] + r * math.cos(theta))
    y = int(pose[1] - r * math.sin(theta))

    return [x, y]


'''
Occupancy Grid Mapping Algorithm
Arguments:
    previous_l_i - previous representation of map with log odds
    current_state - robot's current pose
    current_observations - location of obstacles in our current perceptual field
    map - map matrix (array of elements of clas Cell)
'''
def occupancy_grid_mapping(previous_l_i, current_state, current_observations, map):
    current_l_i = previous_l_i
    new_occupied_pixels = []
    new_free_pixels = []

    for observation in current_observations:
        ending_pixel = distance2pixel(current_state, observation)
        new_occupied_pixels.append(ending_pixel)
        new_free_pixels = bresenham(current_state[:-1], ending_pixel, new_free_pixels) # line does not include ending pixel

    for index, cell in enumerate(map):
        distance = cell.distance_to_state(current_state[:-1]) 
        if distance <= SENSOR_RANGE[0]:
            # current cell is in sensor range
            current_l_i[index] += inverse_sensor_model([cell.x, cell.y], new_occupied_pixels, new_free_pixels, distance)

    draw_map(current_l_i, map)




####################################
# --------------main-------------- #

# Initialize map and log-odds matrix
map_matrix = []
l_i = []

#for i in range(MAP_DIMENSIONS[0]):
 #   for j in range(MAP_DIMENSIONS[1]):
for i in range(MAP_DIMENSIONS[1]):
    for j in range(MAP_DIMENSIONS[0]):
        cell = Cell(i, j)
        map_matrix.append(cell)
        l_i.append(0.0)


# window for occupancy grid map
window = Tk()
window.title("Occupancy Grid Map")
window.geometry(f"{MAP_DIMENSIONS[1]}x{MAP_DIMENSIONS[0]}")

map_window = Canvas(window, width=MAP_DIMENSIONS[1], height=MAP_DIMENSIONS[0], bg="#888A85")
map_window.pack()


# environment graphics
gfx = Graphics(MAP_DIMENSIONS, 'DDR.png', 'NewObstacleMap.png')

# robot
start = (200, 200)
robot = Robot(start, 0.01*3779.52)

# sensor
lidar = Lidar(SENSOR_RANGE, gfx.map)

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
    
    if n <= 40:
        keys = pygame.key.get_pressed() 
        if keys[pygame.K_w]:
            robot.move_forward()
            robot.kinematics(dt)

            robot.x = add_noise(robot.x)
            robot.y = add_noise(robot.y)
        if keys[pygame.K_s]:
            robot.move_backward()
            robot.kinematics(dt)

            robot.x = add_noise(robot.x)
            robot.y = add_noise(robot.y)
        if keys[pygame.K_a]:
            robot.move_left()
            robot.kinematics(dt)

            robot.x = add_noise(robot.x)
            robot.y = add_noise(robot.y)
        if keys[pygame.K_d]:
            robot.move_right()
            robot.kinematics(dt)

            robot.x = add_noise(robot.x)
            robot.y = add_noise(robot.y)


    if n > 40:
        occupancy_grid_mapping(l_i, [robot.x, robot.y, robot.heading], noisy_point_cloud, map_matrix)
        n = 0
    
    gfx.draw_sensor_data(noisy_point_cloud, [robot.x, robot.y, robot.heading])
    
    n += 1

    pygame.display.update()

exit(0)
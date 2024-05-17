import math
import pygame
from tkinter import *
from ROBOTv2 import Graphics, Robot, Lidar


MAP_DIMENSIONS = (600, 1200)


'''
Bresenham's line algorithm
Arguments:
    begin - beginning point
    end - ending point
'''
def bresenham(begin,end):
    x0 = begin[0]
    y0 = begin[1]
    x1 = end[0]
    y1 = end[1]

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
    points.append((x1, y1))
    return points[:-1] # does not return last element


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
Occupancy Grid Mapping Algorithm, with a few tweaks:
    -> We iterate over the 360 (currently) observations, not the cells of the map
    NOTE: the interval between "beams" is 1deg, since the LiDAR's angular resolution is 1deg

    -> For each observation, we apply the Bresenham's algorithm
        --> if we find an obstacle, we color the end point black
        --> otherwise, we color the endpoint, which corresponds to the range of the LiDAR, white

Arguments:
    previous_l_i - previous representation of map with log odds
    current_state - current location of the robot (not pose; we may need to change this to consider angle range smaller than 2*pi)
    current_observations - location of obstacles in our current perceptual field
    sensor_range - ditance (in pixels) and angular (in radians) range of the sensor
'''

def occupancy_grid_mapping(previous_l_i, current_state, current_observations, sensor_range):
    current_l_i = previous_l_i

    for element in current_observations:
        # O facto de a nossa grelha corresponder aos píxeis facilita, pois as coordenadas que calcularmos a partir de element
        # corresponderão a um píxel 
        # Na realidade, teremos de fazer:
                # d = d*meter2grid
                # x_ponto = int(d*cos(teta))
                # y_ponto = int(d*sin(teta))

        # Obtemos, assim, em coordenadas da grid, o ponto [mais próximo] onde se encontra o obstáculo 
        
        if element[0] != -1:
            x_obs = current_state[0] + int(element[0]*math.cos(math.radians(element[1])))
            y_obs = current_state[1] + int(element[0]*math.sin(math.radians(element[1])))
            
            current_l_i[x_obs][y_obs] = 1
        else:
            x_obs =int(current_state[0] + sensor_range[0]*math.cos(math.radians(element[1])))
            y_obs = int(current_state[1] + sensor_range[0]*math.sin(math.radians(element[1])))

            print([x_obs, y_obs])

            current_l_i[x_obs][y_obs] = -1

        # x_obs and y_obs are points in the map reference frame
        line_points = bresenham(current_state,[x_obs, y_obs])
        
        for point in line_points:
            current_l_i[point[0]][point[1]] = -1
    
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
    
    gfx.draw_robot(robot.x, robot.y, robot.heading)

    point_cloud = lidar.sense_obstacles(robot.x, robot.y, robot.heading)

    l_i = occupancy_grid_mapping(l_i, [robot.x, robot.y], point_cloud, sensor_range)
    draw_map(l_i, map_window)
    
    gfx.draw_sensor_data(point_cloud)

    pygame.display.update()
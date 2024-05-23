####################################################################################
# Script to apply to use along with real data:
#   communicates with ROS, subscribing to the required topics;
#   uses AMCL;
#   discretizes the space in evenly spaced square cells. 
#
# Uses a model along the lines of the forward sensor model 
#   (since OccGrid iterates over the observations point cloud and not the map cells)
####################################################################################

##################### A ADAPTAR PARA ROS


import math
#import pygame
from tkinter import *


MAP_DIMENSIONS = (1400, 1400)     # dimensions, in pixels, of the represented map
CELL_SIZE = 0.05    # the length of every environment cell is CELL_SIZE meters


'''
Bresenham's line algorithm for every slope (not just 1)
Arguments:
    begin - coordinates of the beggining pixel
    end - coordinates of the ending pixel

Returns:
    list of all pixels between begin and end , not including 
        the endins pixel
'''
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

    return points # does not include ending pixel


'''
Converts coordinates in the psudo-pixel frame to the real pixel frame.
In the real pixel frame, the origin of the pseudo-pixel frame corresponds to (699, 699).
In the pseudo-pixel frame, the x-axis points to the right and the y-axis points uppwards, 
whereas in the real pixel frame the y-axis points downwards

Arguments:
    pseudo_x - x coordinate in the pseudo-pixel reference frame
    pseudo_y - y coordinate in the pseudo-pixel reference frame

Returns: 
    coordinate of the pixel
'''
def pseudo2pixel(pseudo_x, pseudo_y):
    pixel_x = 699 + pseudo_x
    pixel_y = 699 - pseudo_y

    return [pixel_x, pixel_y]


'''
Transforms circular coordinates (associated to an observation),
in the robot's reference frame, to the coordinates of a pseudo pixel.
We are considering a new pixel reference frame in which the origin corresponds to the center
of the pixel window, x-axis is horizontal to the right, and the y-axis is vertical uppwards

Arguments:
    pose - robot's pose (returned by the AMCL)
    observation - observation under consideration (distance and bearing to the robot)

Returns:
    pixel coordinate
'''
def circular_coordinates2pixel(pose, observation):
    r = observation[0]
    theta = observation[1]
    phi = theta + pose[2] # angle in the world's reference frame
    #phi = observation[1] - robot_heading
    #phi = observation[1]

    # Find point in the world's reference frame
    x = pose[0] + r * math.cos(phi)
    y = pose[1] + r * math.sin(phi)

    # Convert coordinates in the world's reference frame to pixels
    pseudo_pixel_x = int(x/CELL_SIZE)
    pseudo_pixel_y = int(y/CELL_SIZE)

    pixel = pseudo2pixel(pseudo_pixel_x, pseudo_pixel_y)

    return pixel


'''
Draws/Updates occupancy grid map in map_window, considering sensor data anda robot's current pose

Arguments:
    point_clud - sensor data; for each element, [distance, bearing]
    map - window where the map is being drawn (object of class Canvas)
    pose - robot's current pose [x, y, orientation]
'''
def draw_map(point_cloud, map, pose):
    beginning_pixel = circular_coordinates2pixel(pose, [0, 0])

    for observation in point_cloud:
        ending_pixel = circular_coordinates2pixel(pose, observation)
        line = bresenham(beginning_pixel, ending_pixel) # line does not include ending pixel

        for pixel in line:
            # paint the cells between current pose and ending cell (excluding the latter) white (they are empty)
            map.create_rectangle(pixel[0], pixel[1], pixel[0] + 1, pixel[1] + 1, fill="white", outline="")

        # paint the ending cell black (it is occupied)
        map.create_rectangle(ending_pixel[0], ending_pixel[1], ending_pixel[0] + 1, ending_pixel[1] + 1, fill="black", outline="")




########################################
########################################
 # -------------- main -------------- #
#--------------------------------------#


# Window for occupancy grid map
window = Tk()
window.title("Occupancy Grid Map")
window.geometry(f"{MAP_DIMENSIONS[0]}x{MAP_DIMENSIONS[1]}")

map_window = Canvas(window, width=MAP_DIMENSIONS[1], height=MAP_DIMENSIONS[0], bg="#888A85")
map_window.pack()


while running: # while != EOF
    window.update_idletasks()
    window.update()

    robot_pose = get_pose()
    # read sensor data
    point_cloud = lidar.sense_obstacles(robot.x, robot.y, robot.heading)
    draw_map(point_cloud, map_window, robot_pose)
    
    #pygame.display.update()

# keep sowing the map file.....
#!/usr/bin/env python3

####################################################################################
# Script to apply to use along with real data:
#   communicates with ROS, subscribing to the required topics;
#   uses AMCL;
#   discretizes the space in evenly spaced square cells. 
#
# Uses a model along the lines of the forward sensor model 
#   (since OccGrid iterates over the observations point cloud and not the map cells)
####################################################################################


import math
from tkinter import *
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations

################################################################################
# PODEMOS ASSUMIR QUE O ROBOT E O SENSOR ESTÃO NO MESMO REFERNECIAL???????????
################################################################################


class Cell:
    def __init__(self, x, y, value):
        self.x = x
        self.y = y
        self.log_odds = value

    def distance_to_state(self, location):
        return math.sqrt((self.x - location[0])**2 + (self.y - location[1])**2)
    


MAP_DIMENSIONS = [1000, 1000]     # dimensions, in pixels, of the represented map
MAP_CENTER = [499, 699]     # center of the map
CELL_SIZE = 0.05    # the length of every environment cell is CELL_SIZE meters; each cell corresponds to a pixel
LIDAR_ERROR = 0.035     # lidar's uncertainty (in meters)
SENSOR_RANGE = 2.5     # lidar's distance range in meters

global sensor_range_pixels  # lidar's distance range in pixels

sensor_range_pixels = SENSOR_RANGE/CELL_SIZE


# Probability threshold for identifying occupied (upper) and free (lower) cells
p_upper = 0.55 
p_lower = 0.45
UPPER_THRESHOLD = math.log(p_upper/(1-p_upper))
LOWER_THRESHOLD = math.log(p_lower/(1-p_lower))

global p_occ
global p_free

p_occ = 0.68 # P(u-s < X < u+s) = 0.68, for X ~ N(u,s)
p_free = 1-p_occ


global l_i

l_i = []

for x in range(MAP_DIMENSIONS[1]): # x axis
    for y in range(MAP_DIMENSIONS[0]): # y axis
        cell = Cell(x, y, 0.0)
        l_i.append(cell)

global new_occupied_pixels, new_free_pixels

new_occupied_pixels = []
new_free_pixels = []



def scan_callback(data: LaserScan) :
    return data.ranges


def pose_callback(data: PoseWithCovarianceStamped):

    quaternion = [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
              data.pose.pose.orientation.z, data.pose.pose.orientation.w]
 
    euler = tf.transformations.euler_from_quaternion(quaternion)
    
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler[2]]

    return pose


def listener():
    rospy.init_node('subscriber_node', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, pose_callback)
    rospy.spin()


'''
Bresenham's line algorithm for every slope (not just 1)
Arguments:
    begin - coordinates of the beggining pixel
    end - coordinates of the ending pixel

Returns:
    list of all pixels between begin and end , not including 
        the endins pixel
'''
def bresenham(begin, end):
    global new_free_pixels

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
            new_free_pixels.append([x, y])
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            new_free_pixels.append([x, y])
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy

    # does not include last element



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
    pixel_x = MAP_CENTER[0] + pseudo_x
    pixel_y = MAP_CENTER[1] - pseudo_y

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

    # Find point in the world's reference frame
    x = pose[0] + r * math.cos(phi)
    y = pose[1] + r * math.sin(phi)

    # Convert coordinates in the world's reference frame to pixels
    pseudo_pixel_x = int(x/CELL_SIZE)
    pseudo_pixel_y = int(y/CELL_SIZE)

    pixel = pseudo2pixel(pseudo_pixel_x, pseudo_pixel_y)

    return pixel


'''
Computes location of an obstacle in the world frame
Arguments:
    pose - current robot's pose (in the world frame)
    observation - [distance, angle] observed by the robot
'''
def distance2obstacle(pose, observation):
    r = observation[0]
    theta = observation[1] + pose[2]

    x = int(pose[0] + r * math.cos(theta))
    y = int(pose[1] + r * math.sin(theta))

    return [x, y]


'''
Arguments:
    d - distance read by the sensor (in the same unit as SENSOR_RANGE)
'''
def l_occ(d):
    global sensor_range_pixels, p_occ

    w = (sensor_range_pixels - LIDAR_ERROR*d)/sensor_range_pixels
    return math.log(w*p_occ/(1-p_occ))


'''
Arguments:
    d - distance read by the sensor (in the same unit as SENSOR_RANGE)
'''
def l_free(d):
    global sensor_range_pixels, p_free

    w = (sensor_range_pixels + LIDAR_ERROR*d)/sensor_range_pixels
    return math.log(w*p_free/(1-p_free))


'''
Inverse Sensor Model
Arguments:
    pixel - coordinates of the pixel under consideration
    occupied_pixels - list of pixels that are (supposedly) occupied
    free_pixels - list of pixels that are (supposedly) free
    distance - distance between the robot's current location and pixel 
'''
'''
def inverse_sensor_model(pixel, occupied_pixels, free_pixels, distance):
    if pixel in occupied_pixels:
        #return l_occ(distance)
        return 1.0
    
    if pixel in free_pixels:
        #return l_free(distance)
        return -1.0
    
    return 0.0
'''
def inverse_sensor_model(pixel, distance):
    global new_occupied_pixels, new_free_pixels

    if pixel in new_occupied_pixels:
        return l_occ(distance)
        #return 1.0
    
    if pixel in new_free_pixels:
        return l_free(distance)
        #return -1.0
    
    return 0.0


'''
Draw map in map_window
Arguments:
    map - map window
'''
def draw_map(map):
    global l_i

    for cell in l_i:
        if cell.log_odds > UPPER_THRESHOLD:
            # occupied cell
            map.create_rectangle(cell.x, cell.y, cell.x + 1, cell.y + 1, fill="black", outline="")
        elif cell.log_odds < LOWER_THRESHOLD:
            # free cell
            map.create_rectangle(cell.x, cell.y, cell.x + 1, cell.y + 1, fill="white", outline="")


'''
Occupancy Grid Mapping Algorithm
Arguments:
    current_state - robot's current pose (x, y, euler)
    observed_distances - point cloud of the observations (distances to obstacles)
    map - map window
'''
def occupancy_grid_mapping(current_state, observed_distances, map):
    global new_free_pixels, new_occupied_pixels, l_i, sensor_range_pixels

    robot_pixel = circular_coordinates2pixel(current_state, [0, 0])

    new_occupied_pixels.clear()
    new_free_pixels.clear()

    for index, observation in enumerate(observed_distances):
        # Location of the obstacle in the world frame:
        obstacle_location = distance2obstacle(current_state, [observation, math.radians(index)])

        # Location of the obstacle in (real) pixel coordinates:
        obstacle_pixel = circular_coordinates2pixel([obstacle_location[0], obstacle_location[1], 0], [0, 0]) # 0 in the first argument is irrelevant
        
        new_occupied_pixels.append(obstacle_pixel)
        bresenham(robot_pixel, obstacle_pixel)

    for index, cell in enumerate(l_i):
        distance = cell.distance_to_state(robot_pixel) # in pixels
        if distance <= sensor_range_pixels:
            # current cell is in sensor range
            l_i[index].log_odds += inverse_sensor_model([cell.x, cell.y], distance)

    draw_map(map)



########################################
########################################
 # -------------- main -------------- #
#--------------------------------------#
def main():
    global occ2paint, free2paint, painted_occ, painted_free

    # Window for occupancy grid map
    window = Tk()
    window.title("Occupancy Grid Map")
    window.geometry(f"{MAP_DIMENSIONS[0]}x{MAP_DIMENSIONS[1]}")

    map_window = Canvas(window, width=MAP_DIMENSIONS[1], height=MAP_DIMENSIONS[0], bg="#888A85")
    map_window.pack()

    rospy.init_node('subscriber_node', anonymous=True)

    while not rospy.is_shutdown():
        scan_data = rospy.wait_for_message('/scan', LaserScan)
        pose_data = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)

        window.update_idletasks()
        window.update()

        # read sensor data
        observed_ranges = scan_callback(scan_data)
        print(f"{len(observed_ranges) - observed_ranges.count(0.0)} distances")
        # get robot's pose
        robot_pose = pose_callback(pose_data)
        
        occupancy_grid_mapping(robot_pose, observed_ranges, map_window)

    
if __name__ == '__main__':
    main()
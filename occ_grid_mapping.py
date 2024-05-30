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
#import pygame
from tkinter import *
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations
import numpy as np

MAP_DIMENSIONS = [1000, 1000]     # dimensions, in pixels, of the represented map
MAP_CENTER = [499, 699]     # center of the map
CELL_SIZE = 0.05    # the length of every environment cell is CELL_SIZE meters


def scan_callback(data: LaserScan) :
    # Convert the ranges list to a NumPy array
    #ranges_array = np.array(data.ranges)

    # Set the print options for the array
    #np.set_numeric_ops(threshold=np.inf, linewidth=120)

    # Print the array
    #rospy.loginfo("Received scan message with ranges array:")
    #rospy.loginfo(ranges_array)

    #return ranges_array

    return data.ranges


def pose_callback(data: PoseWithCovarianceStamped):

    quaternion = [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
              data.pose.pose.orientation.z, data.pose.pose.orientation.w]
 
    euler = tf.transformations.euler_from_quaternion(quaternion)

    #rospy.loginfo("Received pose message with position: (%f, %f) and orientation yaw=%f",
    #              data.pose.pose.position.x, data.pose.pose.position.y, euler[2])
    
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
Draws/Updates occupancy grid map in map_window, considering sensor data anda robot's current pose

Arguments:
    ranges - sensor data; list of 360 distances (0.0 if no obstacle was found);
        the is a 1deg gap between each observation
    map - window where the map is being drawn (object of class Canvas)
    pose - robot's current pose [x, y, orientation]
'''
def draw_map(ranges, map, pose):
    beginning_pixel = circular_coordinates2pixel(pose, [0, 0])

    count = 0
    for index, distance in enumerate(ranges):
        #if distance > 0.12 and distance < 3.0:
        if distance < 3.5:
            count += 1
            ending_pixel = circular_coordinates2pixel(pose, [distance, math.radians(index)])
            line = bresenham(beginning_pixel, ending_pixel) # line does not include ending pixel

            for pixel in line:
                # paint the cells between current pose and ending cell (excluding the latter) white (they are empty)
                map.create_rectangle(pixel[0], pixel[1], pixel[0] + 1, pixel[1] + 1, fill="white", outline="")

            # paint the ending cell black (it is occupied)
            map.create_rectangle(ending_pixel[0], ending_pixel[1], ending_pixel[0] + 1, ending_pixel[1] + 1, fill="black", outline="")

    print(count)



########################################
########################################
 # -------------- main -------------- #
#--------------------------------------#
def main():
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

        # read sensor data
        ranges = scan_callback(scan_data)
        # get robot's pose
        robot_pose = pose_callback(pose_data)
        
        window.update_idletasks()
        window.update()

        draw_map(ranges, map_window, robot_pose)


    
if __name__ == '__main__':
    main()

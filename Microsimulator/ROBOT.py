import pygame
import math
import numpy as np
import time


# calculates distance between 2 points
def distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1-point2)

class Robot:
    def __init__(self, startpos, width):

        self.m2p = 3779.53 # meters to pixels
        self.w = width

        self.x = startpos[0]
        self.y = startpos[1]
        self.heading = 0

        # velocity of each wheel: 1 cm/s
        self.vl = 0.01*self.m2p
        self.vr = 0.01*self.m2p

        self.maxspeed = 0.02*self.m2p
        self.minspeed = 0.01*self.m2p

        self.min_obs_dist = 100 # pixels
        self.countdown = 5 # seconds
    
    def avoid_obstacles(self, point_cloud, dt):

        closest_obs = None # initial closest obstacle to the robot 
        dist = np.inf # initial distance to the closest obstacle (infinite)

        if len(point_cloud) > 1: # point_cloud not empty; data is arriving from the sensor
            for point in point_cloud:
                if dist > distance([self.x, self.y], point):
                    dist = distance([self.x, self.y], point)
                    closest_obs = (point, dist)

                if closest_obs[1] < self.min_obs_dist and self.countdown > 0:
                    self.countdown -= dt
                    self.move_backward()
                else:
                    self.countdown = 5 # reset countdown
                    self.move_forward()

    def move_backward(self):
        self.vr = - 1.5*self.minspeed 
        self.vl = - 1.5*self.minspeed #/2

    def move_forward(self):
        self.vr = 1.5*self.minspeed
        self.vl = 1.5*self.minspeed

    def move_left(self):
        self.vr =  self.minspeed 
        self.vl = - self.minspeed 

    def move_right(self):
        self.vr = - self.minspeed 
        self.vl =  self.minspeed 

    def kinematics(self, dt):
        self.x += ((self.vl+self.vr)/2) * math.cos(self.heading) * dt
        self.y -= ((self.vl+self.vr)/2) * math.sin(self.heading) * dt # the computer frame is inverted
        
        self.heading += (self.vr - self.vl) / self.w * dt

        if self.heading > 2*math.pi or self.heading < -2*math.pi: 
            self.heading = 0

        self.vr = max(min(self.maxspeed, self.vr), self.minspeed) #limit velocity
        self.vl = max(min(self.maxspeed, self.vl), self.minspeed) #limit velocity

class Graphics:
    def __init__(self, dimensions, robot_img_path, map_img_path):
        pygame.init()

        # COLORS
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        self.yel = (255, 255, 0)

        # ----------MAP----------
        
        # load imgs
        self.robot = pygame.image.load(robot_img_path)
        self.map_img = pygame.image.load(map_img_path)

        # dimensions
        self.height, self.width = dimensions

        # window settings
        pygame.display.set_caption("Turtlebot Mapping - Group 13") #it used to be "Obstacle Avoidance"
        self.map = pygame.display.set_mode((self.width, self.height))
        self.map.blit(self.map_img, (0, 0))

    def draw_robot(self, x, y, heading):
        rotated = pygame.transform.rotozoom(self.robot, math.degrees(heading), 1)
        rect = rotated.get_rect(center = (x, y))
        self.map.blit(rotated, rect)

    def draw_sensor_data(self, point_cloud):
        for point in point_cloud:
            pygame.draw.circle(self.map, self.red, point, 3, 0) #color of obstacle detected

class Lidar:
    def __init__(self, sensor_range, map):
        self.sensor_range = sensor_range
        self.map_width, self.map_height = pygame.display.get_surface().get_size()
        self.map = map

    def sense_obstacles(self, x, y, heading):
        obstacles = []
        x1, y1 = x, y
        start_angle = heading - self.sensor_range[1]
        finish_angle = heading + self.sensor_range[1]
        n_beams = 360
        for angle in np.linspace(start_angle, finish_angle, n_beams, False):
            x2 = x1 + self.sensor_range[0] * math.cos(angle)
            y2 = y1 - self.sensor_range[0] * math.sin(angle)
            n_beam_ponits = 100
            for i in range (0, n_beam_ponits):
                u = i / n_beam_ponits
                x = int(x2*u + x1*(1 - u))
                y = int(y2*u + y1*(1 - u))
                if 0 < x < self.map_width and 0 < y < self.map_height: # sample is inside the map
                    color = self.map.get_at((x, y))
                    self.map.set_at((x, y), (0, 208, 255)) # color of beam 
                    if (color[0], color[1], color[2]) == (0, 0, 0): # there is an obstacle
                        obstacles.append([x, y])
                        break
        return obstacles

"""
class Ultrasonic:
    def __init__(self, sensor_range, map):
        self.sensor_range = sensor_range
        self.map_width, self.map_height = pygame.display.get_surface().get_size()
        self.map = map

    def sense_obstacles(self, x, y, heading):
        obstacles = []
        x1, y1 = x, y
        start_angle = heading - self.sensor_range[1]
        finish_angle = heading + self.sensor_range[1]
        for angle in np.linspace(start_angle, finish_angle, 40, False):
            x2 = x1 + self.sensor_range[0] * math.cos(angle)
            y2 = y1 - self.sensor_range[0] * math.sin(angle)
            for i in range (0, 100):
                u = i / 100
                x = int(x2*u + x1*(1 - u))
                y = int(y2*u + y1*(1 - u))
                if 0 < x < self.map_width and 0 < y < self.map_height: # sample is inside the map
                    color = self.map.get_at((x, y))
                    self.map.set_at((x, y), (0, 208, 255))
                    if (color[0], color[1], color[2]) == (0, 0, 0): # obstacle
                        obstacles.append([x, y])
                        break
        return obstacles
"""

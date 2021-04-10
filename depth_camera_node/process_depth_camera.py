#!/usr/bin/env python

import sys
import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import numpy as np
from lidar.msg import LidarData
from math import sin, cos, radians
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import cv2 as cv
from utils import euler_to_quaternion, Pose, Point
from PIL import Image
from pyquaternion import Quaternion

# Global variable to control if displaying graphs for simulation
DISPLAY_GRAPHS = True
MIN_DISTANCE_FOR_MERGING = 0.5

rover_pose = None
calculation_interval = 100
global_hull_list = set()
MAX_HEIGHT_DIFF = 2
MAX_DISTANCE_TRAVELLED = 500 # in meters
DEPTH_CAMERA_RANGE = 100 # in meters
CELL_SIZE = 0.2 # in meters
GLOBAL_GRID_SIZE = int(MAX_DISTANCE_TRAVELLED / CELL_SIZE)
GLOBAL_GRID_SHAPE = np.array([GLOBAL_GRID_SIZE, GLOBAL_GRID_SIZE])  # For easier calculations later
DEPTH_CAMERA_OFFSET = np.array([2.3, 0, 0])
MAX_HEIGHT = 3

CAMERA_VISION_CONVEX_HULL = []

# Should depend on CAMERA_VISION_CONVEX_HULL (and be a rectangle that the camera convex hull fits inside)
LOCAL_GRID_SIZE = int(DEPTH_CAMERA_RANGE/CELL_SIZE)
LOCAL_GRID_SHAPE = np.array([LOCAL_GRID_SIZE, LOCAL_GRID_SIZE])     # For easier calculations later
global_grid = np.zeros((GLOBAL_GRID_SIZE, GLOBAL_GRID_SIZE), dtype=np.uint8)

print("Global grid using {} megabytes".format(global_grid.nbytes/1000000))
print("Global grid is of shape {}".format(global_grid.shape))
pose_counter = 0
counter = 0

def hull_collision(hull1, hull2):
    # Placeholder for Kieran's function
    return True

def check_if_cell_obstacle(i, j, local_grid):
    # Check all adjacent cells
    for k in range(-1,2):
        for l in range(-1,2):
            if abs(local_grid[i+k,j+l]) > MAX_HEIGHT_DIFF:
                return 255
        
    return 0

def process_point(point, current_pos, rotation, local_grid):

    relative_point_pos = np.array([point.x, point.y, point.z]) - DEPTH_CAMERA_OFFSET
    abs_point_pos = rotation.rotate(relative_point_pos)
    grid_pos = LOCAL_GRID_SHAPE/2 + abs_point_pos[:2]/CELL_SIZE # Center of local grid + number of cells to offset
    grid_pos = grid_pos.astype('int')
    local_grid[grid_pos[0]][grid_pos[1]] = abs_point_pos[2] if abs_point_pos[2] > local_grid[grid_pos[0]][grid_pos[1]] else local_grid[grid_pos[0]][grid_pos[1]]

def local_to_global_grid(grid, current_pos):
    top_left_of_local = GLOBAL_GRID_SHAPE/2 + current_pos[:2]/CELL_SIZE - LOCAL_GRID_SHAPE/2
    top_left_of_local = top_left_of_local.astype('int') 

    for i in range(1, grid.shape[0] - 1):
        for j in range(1, grid.shape[1] - 1):
            if global_grid[top_left_of_local[0] + i][top_left_of_local[1] + j] != 255:
                global_grid[top_left_of_local[0] + i][top_left_of_local[1] + j] = grid[i][j]

def height_to_obstacle(local_height_grid):
    local_obstacle_grid = np.zeros((LOCAL_GRID_SIZE, LOCAL_GRID_SIZE), dtype = np.uint8)
    for i in range(1, local_height_grid.shape[0] - 1):
        for j in range(1, local_height_grid.shape[1] - 1):
            local_obstacle_grid[i][j] = check_if_cell_obstacle(i, j, local_height_grid)

    return local_obstacle_grid

def get_limits(hull):
    min_x = 1000000
    min_y = 1000000
    max_x = -1000000
    max_y = -1000000
    
    return min_x, min_y, max_x, max_y

def hull_center(hull):
    # Returns center of convex_hull through mean of all contour points
    x = 0
    y = 0

    for point in hull:
        point = point[0]
        x += point[0]
        y += point[1]
    
    x /= len(hull)
    y /= len(hull)

    return np.array([x,y])

def process_depth_data(data):
    local_hull_list = []
    local_height_grid = np.zeros((LOCAL_GRID_SIZE, LOCAL_GRID_SIZE), dtype = np.float)
    current_pos = rover_pose.get_current_offset()
    # TODO: Figure out why global grid not working properly, maybe because local grid and pos updated differently??
    # TODO: IS WORKING BUT ROTATION IS APPLYING BEFORE DEPTH DATA~~

    # Create rotation object to apply to all points
    rotation = rover_pose.get_current_inverse_rotation() # Returns quaternion object
    for point in data.points:
        process_point(point, current_pos, rotation, local_height_grid)
    

    local_obstacle_grid = height_to_obstacle(local_height_grid)
    local_to_global_grid(local_obstacle_grid, current_pos)

    if DISPLAY_GRAPHS:
        min_value = np.amin(local_height_grid) # Can be negative
        display_local_grid = cv.resize((local_height_grid + abs(min_value))/MAX_HEIGHT, (500,500))
        cv.imshow("Local Height Grid", display_local_grid)
        cv.waitKey(30)

        cv.imshow("Global Grid", cv.resize(global_grid, (1000,1000)))
        cv.waitKey(300)

    current_pos_cell = current_pos[:2] + GLOBAL_GRID_SHAPE//2

    x_start = current_pos_cell[0] - LOCAL_GRID_SHAPE//2
    x_stop = current_pos_cell[0] + LOCAL_GRID_SHAPE//2

    y_start = current_pos_cell[1] - LOCAL_GRID_SHAPE//2
    y_stop = current_pos_cell[1] + LOCAL_GRID_SHAPE//2

    # TODO: CHANGE list methods to set methods for global_hull_list, make local_hull_list to store references to the elements of global_hull_list which intersect
    local_hull_list = []
    for global_hull in global_hull_list:
        # Maybe sort global_hulls by x coordinate so that don't have to check through whole list only those whose x coordinates could possibly overlap with current rover vision
        if hull_collision(global_hull[0], CAMERA_VISION_CONVEX_HULL): # This means that the hull is currently in vision
            min_x, min_y, max_x, max_y = get_limits(global_hull[0])
            # Then expand the shape of the extended grid to include that hull
            x_start = min(x_start, min_x)
            x_stop = max(x_stop, max_x)

            y_start = min(y_start, min_y)
            y_stop = min (y_stop, max_y)

            # TODO: Check if appends reference or not
            local_hull_list.append(global_hull)

    init_pos = rover_pose.init_pos

    # Get corners of extended grid
    x_start_cell = (x_start - init_pos[0])//CELL_SIZE + GLOBAL_GRID_SIZE//2
    y_start_cell = (y_start - init_pos[1])//CELL_SIZE + GLOBAL_GRID_SIZE//2

    x_stop_cell = (x_stop - init_pos[0])//CELL_SIZE + GLOBAL_GRID_SIZE//2
    y_stop_cell = (y_stop - init_pos[0])//CELL_SIZE + GLOBAL_GRID_SIZE//2
    
    # Take local grid as slice of global grid
    extended_local_grid = global_grid[x_start_cell:x_stop_cell, y_start_cell:y_stop_cell]

    extended_local_grid = cv.convertScaleAbs(extended_local_grid)
    
    black_pixels = np.column_stack(np.where(extended_local_grid==255))
    black_pixels = np.float32(black_pixels)

    if len(black_pixels) > 2:
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.2)
        
        k = 5
        _, labels, centers = cv.kmeans(black_pixels, k, None, criteria, 10, cv.KMEANS_RANDOM_CENTERS)

        # Offset centers according to position of local grid
        extended_local_offset = np.array([x_start_cell, y_start_cell])
        for center in centers:
            center += extended_local_offset
        
        # Remove hulls from global_list that are close enough to k_means centers since these hulls will be added later
        for hull in local_hull_list:
            for center in centers:
                if np.linalg.norm(center - hull[1]) < MIN_DISTANCE_FOR_MERGING:
                    global_hull_list.remove(hull)
                    break
        
        k_means_grid = np.zeros(extended_local_grid.shape)

        color_distinction_offset = 1/(float(k))
        for i, coords in enumerate(black_pixels):
            coords = [int(coords[0]), int(coords[1])]
            label = labels[i]

            for label_number in range(k):
                if label == label_number:
                    k_means_grid[coords[0], coords[1]] = color_distinction_offset * label_number + color_distinction_offset
                    break
        
        if DISPLAY_GRAPHS:
            display_k_means_grid = cv.resize(k_means_grid,(500,500))
            cv.imshow("K means", display_k_means_grid)
            cv.waitKey(300)
        
        for label_number in range(k):
            color = color_distinction_offset * label_number + color_distinction_offset
            lower_bound = color - 0.01
            upper_bound = color + 0.01
            mask = cv.inRange(k_means_grid, lower_bound, upper_bound)
            mask = cv.convertScaleAbs(mask)
            _, contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

            for i in range(len(contours)):
                hull = cv.convexHull(contours[i], returnPoints=True)
                hull += (current_pos[:2]//CELL_SIZE).astype(np.int32) + GLOBAL_GRID_SHAPE//2
                # Append hull with its center
                global_hull_list.add((hull, hull_center(hull)))
        print("Hull List", global_hull_list)
        # hull_points = np.zeros((GRID_SIZE + 1, GRID_SIZE+ 1, 3), dtype = np.uint8)

        # # To display points of convex hulls
        # # for point in hull_list[1]:
        # #     point = point[0]
        # #     hull_points[point[1]][point[0]] = 255

        # hull_points = cv.resize(hull_points, (500, 500))
        # cv.imshow("Hull points", hull_points)
        # cv.waitKey(0)
        if DISPLAY_GRAPHS:
            hulls = np.zeros((GLOBAL_GRID_SIZE, GLOBAL_GRID_SIZE, 3), dtype = np.uint8)
            cv.drawContours(hulls, global_hull_list, -1, (255,255,255))
            cv.imshow("Convex hulls", cv.resize(hulls, (1000, 1000)))
            cv.waitKey(300)

        
        
# Callback called everytime subscriber receives data on depth_camera_point_cloud topic
def depth_camera_callback(data):
    global counter
    counter += 1
    if counter >= calculation_interval:
        counter = 0
        process_depth_data(data) 

def rover_pose_callback(data):
    global rover_pose
    if rover_pose == None:
        rover_pose = Pose(data.position, data.orientation)

    rover_pose.update_current_pos(data.position)
    rover_pose.update_current_quaternion(data.orientation)

def listener():
    rospy.init_node('process_lidar', anonymous=True)

    rospy.Subscriber('/depth_camera_point_cloud', sensor_msgs.msg.PointCloud, depth_camera_callback)
    rospy.Subscriber('/rover_pose', geometry_msgs.msg.Pose, rover_pose_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

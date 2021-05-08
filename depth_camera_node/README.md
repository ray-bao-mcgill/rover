# Files

- utils.py --> Contains rover Pose class and helper functions for rotations
- GJK.py --> Contains collision code from Kieran as well as helper classes for that collision code
- process_lidar.py --> Contains most of the main logic

# Steps in mapping process
## Setup
- In listener(), initialize the node and initialize the subscribers to the "/depth_camera_point_cloud" and "/rover_pose" topics. 

## Processing pose data
- When a message is received on the "/rover_pose" topic, rover_pose_callback() is called, which updates the current rover_pose

- The Pose class has 2 main useful methods:
  - get_current_offset() returns the offset from the rover's initial position
  - get_current_inverse_rotation() returns the rotation offset from the rover's initial rotation
- These methods are used to keep most calculations relative to the rover's initial position and rotation

## Processing depth data
- When a message is received on the "/depth_camera_point_cloud" topic, depth_camera_callback() is called
- This method keeps a global counter and only does the calculations (in process_depth_data()) once the counter is greater or equal to the calculation_interval parameter

## Local height grid
- In process_depth_data(), the first step is to get a local height grid. The rover is always at the center of the local height grid (using get_current_offset()) and the local grid is always pointing in the same direction as the rover's initial direction (using get_current_inverse_rotation())
- All points in the point cloud are processed using process_point(): their corresponding cell in the local height grid is found, then that cell's height is set to the height of the point (after it has been modified according to the rover's current height and rotation)

## Local obstacle/occupancy grid
- The local height grid is then transformed into a local obstacle grid using the height_to_obstacle() function
- height_to_obstacle() goes through all cells in the local height grid and compares their value to the adjacent cells. If the max height difference between a cell and one of its adjacent cells is more than the MAX_HEIGHT_DIFF parameter, the cell is determined to contain an obstacle.
- There may be more efficient ways to perform this step

## Global grid
- A global grid is initialized at the start of the script.
- Every time a local obstacle grid is computed, its information is transferred into the global grid using local_to_global_grid()

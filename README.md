## DRONE WAYPOINT MANAGEMENT ##

This algorithm inputs a geojson-list of waypoints, which consist of latitude, longitude and terrain altitude, 
as well of as a desired drone altitude above the ground. It then extracts the minimum amount of coordinates needed for a drone
to follow this path, with any given altitude above the terrain, without crashing into the ground.  

# How to run the algorithm #
Execute extract_waypoints.py in Python

# Required Packages #
The following packages need to be installed in order for the algorithm to work properly:
- math
- matplotlib
- numpy
- json 

# How the algorithm works #

Since a linear, straight path is assumed, the 3D-space is converted into a 2D-space, consisting of a distance from the 
start point (x) and an altitude (y). An ideal path is then calculated as the sum of the terrain coordinates and 
the desired altitude. The minimum amount of coordinates is then gives a the concatinated array of the following 
two coordinate lists:
   - extreme_points, which is all the coordinates located in a local minima- or maxima in the ideal drone path
   - mid_points, which is the all coordinates that have the largest positive perpendicular distance to the straight line between the two 
   closest surrounding extreme point
   
If the number of coordinates are considered more important than to follow the altitude, the extreme_points can easily be 
be modified to only include local maxima points.
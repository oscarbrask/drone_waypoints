import math
import matplotlib.pyplot as plt
import numpy as np

def calculate_coordinate_distance(coord1, coord2):
    '''returns the distance between a pair of coordinates on earth'''
    R_earth = 6373.0

    lat1 = math.radians(coord1[1])
    lon1 = math.radians(coord1[0])
    lat2 = math.radians(coord2[1])
    lon2 = math.radians(coord2[0])

    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R_earth * c
    return distance


def calculate_point_distance(points1, points2):
    '''returns the distance between teo points in a regular coordinate space'''
    x1 = points1[0]
    x2 = points2[0]
    y1 = points1[1]
    y2 = points2[1]
    return (((x2 - x1) ** 2) + ((y2 - y1) ** 2)) ** 0.5


def calculate_point_to_line_distance(line_point_1, line_point_2, point):
    '''returns the perpendicular distance between a point and a line defined by two points'''
    return np.abs(np.linalg.norm(np.cross(line_point_2 - line_point_1, line_point_1 -point))) / \
           np.linalg.norm(line_point_2 - line_point_1)


def extract_distances(coord_list, start_coords):
    '''returns a list of distances from a start points to each point in a list of coordinates'''
    distances = []
    for i in range(len(coord_list)):
        distance = calculate_coordinate_distance(coord_list[i][0:2], start_coords)
        distances.append(distance)
    return distances


def create_figures(path_array, terrain_array, mid_points, extreme_points, final_coordinates):
    '''plots stuff'''
    # create distance vectors
    distances = extract_distances(path_array, path_array[0][0:2])  # ideal path
    distances_mp = extract_distances(mid_points, path_array[0][0:2])  # mid points
    distances_xp = extract_distances(extreme_points, path_array[0][0:2])  # extreme points
    distances_final = extract_distances(final_coordinates, path_array[0][0:2])  # final points

    # plot distance-altitude with seperated mid- and extreme points
    plot1 = plt.figure(1)
    plt.plot(distances, path_array[:, 2])
    plt.plot(distances, terrain_array[:, 2])
    plt.scatter(distances_xp, extreme_points[:, 2])
    plt.scatter(distances_mp, mid_points[:, 2])
    plt.xlabel('Distance from start')
    plt.ylabel('Altitude')
    plt.legend(['Ideal Drone Path', 'Terrain', 'Extreme Points', 'Mid Points'])

    # plot long-lat
    plot2 = plt.figure(2)
    plt.scatter(path_array[:, 0], path_array[:, 1])
    plt.xlabel('Lon')
    plt.ylabel('Lat')

    # plot distance-altitude with the final coordinate path
    plot3 = plt.figure(3)
    plt.plot(distances, path_array[:, 2])
    plt.plot(distances, terrain_array[:, 2])
    plt.scatter(distances_final, final_coordinates[:, 2])
    plt.plot(distances_final, final_coordinates[:, 2])
    plt.xlabel('Distance from start')
    plt.ylabel('Altitude')
    plt.legend(['Ideal Drone Path', 'Terrain', 'Actual Drone Path', 'Drone Coordinates'])

    plt.show()

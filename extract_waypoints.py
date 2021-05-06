import json
import numpy as np
from support_functions import calculate_coordinate_distance, calculate_point_to_line_distance, create_figures


def main(file_path, desired_elevation):

    # ### EXTRACT TERRAIN FROM JSON ### #
    with open(file_path) as f:
        gj = json.load(f)

    terrain_array = np.zeros((len(gj['features']), 3))
    for index, row in enumerate(terrain_array):
        terrain_array[index][0] = gj['features'][index]['geometry']['coordinates'][0]  # long
        terrain_array[index][1] = gj['features'][index]['geometry']['coordinates'][1]  # lat
        terrain_array[index][2] = gj['features'][index]['properties']['terrain']       # altitude

    # ### EXTRACT IDEAL DRONE PATH ### #
    path_array = terrain_array.copy()
    path_array[:, 2] += desired_elevation

    # ### FIND EXTREME POINTS #### #
    extreme_points = np.empty((0, 4))

    # loop over points in path_array, except start- and end point
    for row_index in range(1, len(path_array)-1):
        prev = path_array[row_index-1]
        curr = path_array[row_index]
        next = path_array[row_index+1]

        # check local min- or max
        is_local_max = (curr[2] > prev[2]) and (curr[2] > next[2])
        is_local_min = (curr[2] < prev[2]) and (curr[2] < next[2])

        # add start point to extreme_points
        if row_index == 1:
            extreme_points = np.append(extreme_points, np.array([[prev[0], prev[1], prev[2], row_index-1]]), axis=0)

        # if current point is extreme point, add to extreme_points
        elif is_local_max or is_local_min:
            extreme_points = np.append(extreme_points, np.array([[curr[0], curr[1], curr[2], row_index]]), axis=0)

        # add end point to extreme_points
        elif row_index == len(path_array)-2:
            extreme_points = np.append(extreme_points, np.array([[next[0], next[1], next[2], row_index+1]]), axis=0)

    # ### FIND MID POINTS #### #
    mid_points = np.empty((0, 4))

    # loop over pair of extreme points
    for i in range(len(extreme_points)-1):

        # define a line between the pair of extreme points l1 and l2
        l1_coords = extreme_points[i, 0:2]
        l2_coords = extreme_points[i + 1, 0:2]
        l1_y = extreme_points[i, 2]
        l2_y = extreme_points[i + 1, 2]

        # define equation for this line
        m = l1_y
        dy = l2_y - l1_y
        dx = calculate_coordinate_distance(l2_coords, l1_coords)
        k = dy/dx

        # extract coordinates in coordinate space [distance, altitude]
        l1_dxdy = np.array([0, 0])
        l2_dxdy = np.array([dx, dy])

        # perpendicular distance from point to line
        d_perp_max = 0

        # loop over all points in path between extreme points
        index = int(extreme_points[i, 3])
        index_next = int(extreme_points[i+1, 3])
        for j in range(index, index_next):

            # extract point coordinates in space [distance, altitude]
            p_y = path_array[j, 2]
            p_coords = path_array[j, 0:2]
            p_dx = calculate_coordinate_distance(p_coords, l1_coords)
            p_dy = p_y - l1_y
            p_dxdy = np.array([p_dx, p_dy])
            y = k * p_dx + m
            p_is_above_line = p_y - y > 0

            # calculate perpendicular distance to line
            d_perp = abs(calculate_point_to_line_distance(l1_dxdy, l2_dxdy, p_dxdy))

            # find the point between the extreme points with largest perpendicular distance to line
            if d_perp > d_perp_max and p_is_above_line:
                mid_point_candidate = np.array([[p_coords[0], p_coords[1], p_y, j]])
                d_perp_max = d_perp

        # if this point exist, add to list of mid_points
        if hasattr(mid_point_candidate, 'shape'):
            mid_points = np.append(mid_points, mid_point_candidate, axis=0)

    # concatenate extreme points and mid points to final coordinate list
    final_coordinates = np.concatenate((extreme_points, mid_points), axis=0)
    final_coordinates = final_coordinates[np.argsort(final_coordinates[:, 0])]

    # create plots
    create_figures(path_array, terrain_array, mid_points, extreme_points, final_coordinates)

    return final_coordinates[:, 0:3]


if __name__ == '__main__':
    desired_altitude = 10
    file_path = 'assessment.geojson'
    list_of_coordinates = main(file_path, desired_altitude)
    print(list_of_coordinates)

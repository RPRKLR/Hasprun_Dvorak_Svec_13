# Simple example of BFS algorithm adapted from https://github.com/shkolovy/path-finder-algorithms          

from pprint import pprint
from collections import deque
import copy
import numpy as np
from PIL import Image
import numpy as np
from math import atan2


START_COL = "S"
END_COL = "E"
VISITED_COL = "x"
OBSTACLE_COL = "#"
PATH_COL = "@"

def scan_grid(grid, start=(0, 0)):
    """Scan all grid, so we can find a path from 'start' to any point"""

    q = deque()
    q.append(start)
    came_from = {start: None}
    while len(q) > 0:
        current_pos = q.popleft()
        neighbors = get_neighbors(grid, current_pos[0], current_pos[1])
        for neighbor in neighbors:
            if neighbor not in came_from:
                q.append(neighbor)
                came_from[neighbor] = current_pos

    return came_from

def get_neighbors(grid, row, col):
    height = len(grid)
    width = len(grid[0])

    neighbors = [(row + 1, col), (row, col - 1), (row - 1, col), (row, col + 1)]

    # make path nicer
    if (row + col) % 2 == 0:
        neighbors.reverse()

    # check borders
    neighbors = filter(lambda t: (0 <= t[0] < height and 0 <= t[1] < width), neighbors)
    # check obstacles
    neighbors = filter(lambda t: (grid[t[0]][t[1]] != OBSTACLE_COL), neighbors)

    return neighbors

def parse_pgm(data):
    lines = data.split("\n")
    metadata = {}
    pixel_data = []

    # Loop through lines and parse data
    for line in lines:
        # Skip comments
        if line.startswith("#"):
            continue
        # Check for magic number P2
        elif line == "P2":
            metadata["type"] = "P2"
        # Check for width and height
        elif "width" not in metadata:
            metadata["width"], metadata["height"] = map(int, line.split())
        # Check for max gray value
        elif "max_gray" not in metadata:
            metadata["max_gray"] = int(line)
        # Parse pixel data
        else:
            pixel_data.append(list(map(int, line.split())))
    return metadata, pixel_data

def replace_values_in_array(pixel_data):
    for i in range(len(pixel_data)):
        for j in range(len(pixel_data[i])):
            if pixel_data[i][j] == 255:
                pixel_data[i][j] = '.'
            elif pixel_data[i][j] == 0:
                pixel_data[i][j] = '#'
    return pixel_data

def write_2d_array_to_file(pixel_data, filename):
    max_width = max(len(str(item)) for row in pixel_data for item in row)  # Find the maximum width of the items
    with open(filename, 'w') as file:
        for row in pixel_data:
            # Create a formatted string with even spacing, write it to the file
            line = ''.join(f'{item:>{max_width+1}}' for item in row)
            file.write(line + '\n')

def write_pgm(pixel_data, filename, max_value=255):
    # Ensure max_value is valid
    max_value = min(max(max_value, 0), 255)

    # Determine the dimensions of the image
    height = len(pixel_data)
    width = len(pixel_data[0]) if height > 0 else 0

    # Write header and pixel data to file
    with open(filename, 'w') as f:
        f.write(f"P2\n{width} {height}\n{max_value}\n")
        for row in pixel_data:
            f.write(' '.join(map(str, row)) + '\n')

def convert_to_numeric(pixel_data):
    """
    Convert a 2D array of '.' and '#' symbols to a 2D array of 0 and 255 values, respectively.

    :param pixel_data: 2D array containing '.' and '#' symbols.
    :return: A new 2D array with numerical values.
    """
    return [[255 if pixel == '.' else 0 for pixel in row] for row in pixel_data]

def find_path(start, end, came_from):
    """Find the shortest path from start to end point"""

    path = [end]

    current = end
    while current != start:
        print("CURRENT: " + str(current))
        current = came_from[current]
        path.append(current)

    # reverse to have Start -> Target
    # just looks nicer
    path.reverse()

    return path

def draw_path(path, grid):
    for row, col in path:
        grid[row][col] = PATH_COL

    # draw start and end
    start_pos = path[0]
    end_pos = path[-1]
    grid[start_pos[0]][start_pos[1]] = START_COL
    grid[end_pos[0]][end_pos[1]] = END_COL

    return grid

def make_safety_area_for_obstacles(grid):
    for idx_col, col in enumerate(grid):
        for idx_row, row in enumerate(col):
            if(row == 255):
                continue
            elif(row == 0):
                for i in range(-3, 3, 1):
                    for j in range(-3, 3, 1):
                        if(idx_col + i < 0 or idx_row + j < 0 or idx_col + i > len(grid) - 1 or idx_row + j > len(grid[-1]) - 1):
                            continue
                        if (grid[idx_col + i][idx_row + j] == 0):
                            continue
                        else:
                            grid[idx_col + i][idx_row + j] = 1
                            print(grid[idx_col + i][idx_row + j])
    print(grid)

    for idx_col, col in enumerate(grid):
        for idx_row, row in enumerate(col):
            if(grid[idx_col][idx_row] == 1):
                grid[idx_col][idx_row] = 0
    print(grid)
    return grid

def perpendicular_distance(point, line_start, line_end):
    # Calculate the perpendicular distance between a point and a line segment
    if np.array_equal(line_start, line_end):
        return np.linalg.norm(point - line_start)
    
    line_length = np.linalg.norm(line_end - line_start)
    t = np.dot(point - line_start, line_end - line_start) / (line_length ** 2)
    
    if t < 0:
        return np.linalg.norm(point - line_start)
    if t > 1:
        return np.linalg.norm(point - line_end)
    
    projection = line_start + t * (line_end - line_start)
    return np.linalg.norm(point - projection)

def douglas_peucker(points, epsilon):
    if len(points) <= 2:
        return points
    
    # Find the point with the maximum distance
    max_distance = 0
    max_index = 0
    
    line_start = points[0]
    line_end = points[-1]
    
    for i in range(1, len(points) - 1):
        distance = perpendicular_distance(points[i], line_start, line_end)
        if distance > max_distance:
            max_distance = distance
            max_index = i
    
    if max_distance > epsilon:
        # Recursively simplify the two sub-paths
        recursive_start = douglas_peucker(points[:max_index+1], epsilon)
        recursive_end = douglas_peucker(points[max_index:], epsilon)
        
        # Combine the two sub-paths
        simplified = recursive_start[:-1] + recursive_end
    else:
        simplified = [line_start, line_end]
    
    return simplified


def init(map_name, altitude, start_x, start_y, end_x, end_y):
    with open(map_name + str(altitude) + ".pgm", "rb") as file:
        byte_data = file.read()
        data = byte_data.decode("utf-8")

    metadata, pixel_data = parse_pgm(data)

    pixel_data = replace_values_in_array(pixel_data)
    filtered_data = [sublist for sublist in pixel_data if sublist]    

    filtered_data_pgm = convert_to_numeric(filtered_data)
    write_pgm(filtered_data_pgm, 'map.pgm')


    extrapolated_obstacle_grid =  make_safety_area_for_obstacles(filtered_data_pgm)
    print(extrapolated_obstacle_grid)

    write_pgm(extrapolated_obstacle_grid, 'extrapolated.pgm')

    # reading the data from the extrapolated map
    with open("extrapolated.pgm", "rb") as file:
        byte_data = file.read()
        data = byte_data.decode("utf-8")

    metadata, pixel_data = parse_pgm(data)

    pixel_data = replace_values_in_array(pixel_data)
    filtered_data = [sublist for sublist in pixel_data if sublist]    


    start_pos = (start_x, start_y)
    end_pos = (end_x, end_y)
    directions = scan_grid(filtered_data, start_pos)

    path1 = find_path(start_pos, end_pos, directions)

    points = np.array(path1)
    simplified_points = douglas_peucker(points, 1.0)
    print(simplified_points)

    grid_with_path1 = draw_path(path1, copy.deepcopy(filtered_data))

    grid_with_path1_converted = convert_to_numeric(grid_with_path1)

    write_pgm(grid_with_path1_converted, 'nonfiltered_path_points.pgm')
    grid_with_path1 = draw_path(simplified_points, copy.deepcopy(filtered_data))

    grid_with_path1_converted = convert_to_numeric(grid_with_path1)

    write_pgm(grid_with_path1_converted, 'filtered_path_output.pgm')
    





if __name__ == "__main__":
    init(map_name="map_", altitude=125, start_x=250, start_y=300, end_x=50, end_y=35)
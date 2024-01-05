# Simple example of BFS algorithm adapted from https://github.com/shkolovy/path-finder-algorithms          

from pprint import pprint
from collections import deque
import copy
import numpy as np
import math
import csv
import sys
from queue import PriorityQueue

START_COL = "S"
END_COL = "E"
VISITED_COL = "x"
OBSTACLE_COL = "#"
PATH_COL = "@"
ROW = 450
COL = 450

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

def convert_to_numeric_ones(pixel_data):
    return [[1 if pixel == '.' else 0 for pixel in row] for row in pixel_data]

def find_path(start, end, came_from):
    """Find the shortest path from start to end point"""

    path = [end]

    current = end
    while current != start:
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
    offset = 3
    for idx_col, col in enumerate(grid):
        for idx_row, row in enumerate(col):
            if(row == 255):
                continue
            elif(row == 0):
                for i in range(-offset-1, offset+1, 1):
                    for j in range(-offset-1, offset+1, 1):
                        if(idx_col + i < 0 or idx_row + j < 0 or idx_col + i > len(grid) - 1 or idx_row + j > len(grid[-1]) - 1):
                            continue
                        if (grid[idx_col + i][idx_row + j] == 0):
                            continue
                        else:
                            grid[idx_col + i][idx_row + j] = 1
                            # print(grid[idx_col + i][idx_row + j])
    # print(grid)

    for idx_col, col in enumerate(grid):
        for idx_row, row in enumerate(col):
            if(grid[idx_col][idx_row] == 1):
                grid[idx_col][idx_row] = 0
    # print(grid)
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


def save_points_to_csv(points, csv_name):
    with open(csv_name, 'w', newline='') as file:
        writer = csv.writer(file)
        for point in points:
            writer.writerow(point)


# A structure to hold the necessary parameters
class Cell:
    def __init__(self, parent_i=-1, parent_j=-1, f=float('inf'), g=float('inf'), h=float('inf')):
        self.parent_i = parent_i
        self.parent_j = parent_j
        self.f = f
        self.g = g
        self.h = h

# A Utility Function to check whether given cell (row, col)
# is a valid cell or not.
def is_valid(row, col):
    # Returns true if row number and column number
    # are in range
    return 0 <= row < ROW and 0 <= col < COL

# A Utility Function to check whether the given cell is
# blocked or not
def is_unblocked(grid, row, col):
    # Returns true if the cell is not blocked else false
    return grid[row][col] == 1

# A Utility Function to calculate the 'h' heuristics.
def calculate_h_value(row, col, dest):
    # Return using the distance formula
    return math.sqrt((col - dest[1]) ** 2 + (row - dest[0]) ** 2)

# A Function to find the shortest path between
# a given source cell to a destination cell according
# to A* Search Algorithm
def a_star_search(grid, src, dest):
    # If the source is out of range
    if not is_valid(src[0], src[1]):
        print("Source is invalid")
        return

    # If the destination is out of range
    if not is_valid(dest[0], dest[1]):
        print("Destination is invalid")
        return

    # Either the source or the destination is blocked
    if not is_unblocked(grid, src[0], src[1]) or not is_unblocked(grid, dest[0], dest[1]):
        print("Source or the destination is blocked")
        return

    # If the destination cell is the same as source cell
    if src == dest:
        print("We are already at the destination")
        return

    # Create a closed list and initialise it to false which
    # means that no cell has been included yet. This closed
    # list is implemented as a boolean 2D array.
    closed_list = [[False for _ in range(COL)] for _ in range(ROW)]

    # Declare a 2D array of structure to hold the details
    # of that cell
    cell_details = [[Cell() for _ in range(COL)] for _ in range(ROW)]

    # Initialize parameters of the starting node
    i, j = src
    cell_details[i][j] = Cell(i, j, 0.0, 0.0, 0.0)

    # Create an open list using PriorityQueue
    open_list = PriorityQueue()

    # Put the starting cell on the open list and set its 'f' as 0
    open_list.put((0.0, src))

    # We set this boolean value as false as initially
    # the destination is not reached.
    found_dest = False

    while not open_list.empty():
        f, (i, j) = open_list.get()

        # Add this vertex to the closed list
        closed_list[i][j] = True

        # Generating all the 4 successors of this cell
        successors = [
            (-1, 0), (1, 0), (0, 1), (0, -1)
        ]

        for dx, dy in successors:
            new_i, new_j = i + dx, j + dy

            if is_valid(new_i, new_j):
                # If the destination cell is the same as the current successor
                if (new_i, new_j) == dest:
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j
                    print("The destination cell is found")
                    output_path = trace_path(cell_details, dest)
                    found_dest = True
                    return output_path

                # If the successor is not on the closed list and is unblocked
                if not closed_list[new_i][new_j] and is_unblocked(grid, new_i, new_j):
                    g_new = cell_details[i][j].g + 1.0
                    h_new = calculate_h_value(new_i, new_j, dest)
                    f_new = g_new + h_new

                    # If the successor is not in the open list or the new f is less
                    if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                        open_list.put((f_new, (new_i, new_j)))

                        # Update the details of this cell
                        cell_details[new_i][new_j].f = f_new
                        cell_details[new_i][new_j].g = g_new
                        cell_details[new_i][new_j].h = h_new
                        cell_details[new_i][new_j].parent_i = i
                        cell_details[new_i][new_j].parent_j = j

    # When the destination cell is not found and the open
    # list is empty, then we conclude that we failed to
    # reach the destination cell.
    if not found_dest:
        print("Failed to find the Destination Cell")

# A Utility Function to trace the path from the source
# to destination
def trace_path(cell_details, dest):
    print("\nThe Path is ", end="")
    row, col = dest

    path = []
    output_path = []
    while cell_details[row][col].parent_i != row or cell_details[row][col].parent_j != col:
        path.append((row, col))
        temp_row, temp_col = cell_details[row][col].parent_i, cell_details[row][col].parent_j
        row, col = temp_row, temp_col

    path.append((row, col))
    while path:
        p = path.pop()
        print(f"-> {p}", end=" ")
        output_path.append(p)
    print()
    return output_path

def init(map_name, altitude, start_x, start_y, end_x, end_y, csv_name):
    print("/home/lrs-ubuntu/LRS/Hasprun_Dvorak_13/src/LRS-FEI-main/scripts/" + map_name + "" + altitude + ".pgm")
    with open("/home/lrs-ubuntu/LRS/Hasprun_Dvorak_13/src/LRS-FEI-main/scripts/" + map_name + "" + altitude + ".pgm", "rb") as file:
        byte_data = file.read()
        data = byte_data.decode("utf-8")

    metadata, pixel_data = parse_pgm(data)

    pixel_data = replace_values_in_array(pixel_data)
    filtered_data = [sublist for sublist in pixel_data if sublist]    

    filtered_data_pgm = convert_to_numeric(filtered_data)
    write_pgm(filtered_data_pgm, 'map.pgm')


    extrapolated_obstacle_grid =  make_safety_area_for_obstacles(filtered_data_pgm)
    # print(extrapolated_obstacle_grid)

    write_pgm(extrapolated_obstacle_grid, 'extrapolated.pgm')

    # reading the data from the extrapolated map
    with open("extrapolated.pgm", "rb") as file:
        byte_data = file.read()
        data = byte_data.decode("utf-8")

    metadata, pixel_data = parse_pgm(data)

    pixel_data = replace_values_in_array(pixel_data)
    filtered_data = [sublist for sublist in pixel_data if sublist]    

    src = (start_x, start_y)
    dest = (end_x, end_y)


    grid = convert_to_numeric_ones(filtered_data)

    # print(grid)

    path1 = a_star_search(grid, src, dest)

    # path1 = find_path(start_pos, end_pos, directions)

    points = np.array(path1)
    simplified_points = douglas_peucker(points, 1)
    
    grid_with_path1 = draw_path(path1, copy.deepcopy(filtered_data))

    grid_with_path1_converted = convert_to_numeric(grid_with_path1)

    write_pgm(grid_with_path1_converted, 'nonfiltered_path_points.pgm')
    grid_with_path1 = draw_path(simplified_points, copy.deepcopy(filtered_data))

    grid_with_path1_converted = convert_to_numeric(grid_with_path1)

    write_pgm(grid_with_path1_converted, 'filtered_path_output.pgm')

    save_points_to_csv(simplified_points, csv_name)



def main(argv):
    print(argv[0:])
    init(map_name=argv[0], altitude=argv[1], start_x=int(argv[2]), start_y=int(argv[3]), end_x=int(argv[4]), end_y=int(argv[5]), csv_name=argv[6])

    

if __name__ == "__main__":
    # init(map_name="map_", altitude=125, start_x=250, start_y=300, end_x=50, end_y=35, csv_name="simplified_points.csv")
    main(sys.argv[1:])
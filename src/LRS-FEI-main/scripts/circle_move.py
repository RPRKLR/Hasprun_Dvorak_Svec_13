# Simple example of BFS algorithm adapted from https://github.com/shkolovy/path-finder-algorithms          

from pprint import pprint
from collections import deque
import copy
import numpy as np
import math
import csv
import sys
from queue import PriorityQueue



def save_points_to_csv(points, csv_name):
    with open(csv_name, 'w', newline='') as file:
        writer = csv.writer(file)
        for point in points:
            writer.writerow(point)


def calculate_circle_points(src, radius=15, num_points=7):
    start_y, start_x = src
    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    x_coords = start_x + radius * np.cos(angles)
    y_coords = start_y + radius * np.sin(angles)
    #
    # points = list(zip(y_coords, x_coords))
    first_point = None
    points = list(zip(x_coords, y_coords))
    output_path = []

    # print("\n----- CIRCLE ------")
    # print("\nThe Path is ", end="")
    for i, p in enumerate(points):
        if i == 0:
            first_point = p
        # print(f"-> {p} ", end=" ")
        output_path.append(p)

    # output_path.append((start_y, start_x))
    output_path.append(first_point)
    output_path.append((start_x, start_y))

    # print(f"{output_path}")

    return output_path

# A Utility Function to check whether given cell (row, col)
# is a valid cell or not.

def init(map_name, altitude, start_x, start_y, end_x, end_y, csv_name):

    src = (start_y, start_x)

    path1 = calculate_circle_points(src)
    points = np.array(path1)
  

    save_points_to_csv(points, csv_name)



def main(argv):
    print(argv[0:])
    init(map_name=argv[0], altitude=argv[1], start_x=int(argv[2]), start_y=int(argv[3]), end_x=int(argv[4]), end_y=int(argv[5]), csv_name=argv[6])

    

if __name__ == "__main__":
    # init(map_name="map_", altitude=125, start_x=250, start_y=300, end_x=50, end_y=35, csv_name="simplified_points.csv")
    main(sys.argv[1:])
import math
from queue import PriorityQueue
from pprint import pprint
from collections import deque
import copy
import numpy as np
from PIL import Image
import csv
import sys



ROW = 9
COL = 10

Pair = tuple[int, int]

class Cell:
    def __init__(self):
        self.parent_i = -1
        self.parent_j = -1
        self.f = math.inf
        self.g = math.inf
        self.h = math.inf

def is_valid(row, col):
    return 0 <= row < ROW and 0 <= col < COL

def is_unblocked(grid, row, col):
    return grid[row][col] != 0  # Consider 0 as a blocked cell

def is_destination(row, col, dest):
    return row == dest[0] and col == dest[1]

def calculate_h_value(row, col, dest):
    return math.sqrt((row - dest[0]) ** 2 + (col - dest[1]) ** 2)

def trace_path(cell_details, dest):
    print("\nThe Path is ", end="")
    row, col = dest
    path = []

    while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
        path.append((row, col))
        temp_row = cell_details[row][col].parent_i
        temp_col = cell_details[row][col].parent_j
        row = temp_row
        col = temp_col
    full_path = []

    path.append((row, col))
    while path:
        p = path.pop()
        print(f"-> {p}", end=" ")
        full_path.append(p)
    print()
    return full_path

def a_star_search(grid, src, dest):
    if not is_valid(src[0], src[1]):
        print("Source is invalid")
        return

    if not is_valid(dest[0], dest[1]):
        print("Destination is invalid")
        return

    if not is_unblocked(grid, src[0], src[1]) or not is_unblocked(grid, dest[0], dest[1]):
        print("Source or the destination is blocked")
        return

    if is_destination(src[0], src[1], dest):
        print("We are already at the destination")
        return

    closed_list = [[False for _ in range(COL)] for _ in range(ROW)]
    cell_details = [[Cell() for _ in range(COL)] for _ in range(ROW)]

    for i in range(ROW):
        for j in range(COL):
            cell_details[i][j].f = math.inf
            cell_details[i][j].g = math.inf
            cell_details[i][j].h = math.inf
            cell_details[i][j].parent_i = -1
            cell_details[i][j].parent_j = -1

    i, j = src
    cell_details[i][j].f = 0.0
    cell_details[i][j].g = 0.0
    cell_details[i][j].h = 0.0
    cell_details[i][j].parent_i = i
    cell_details[i][j].parent_j = j

    open_list = PriorityQueue()
    open_list.put((0.0, src))

    found_dest = False

    while not open_list.empty():
        p = open_list.get()
        i, j = p[1]
        closed_list[i][j] = True

        if is_destination(i, j, dest):
            print("The destination cell is found")
            full_path = trace_path(cell_details, dest)
            found_dest = True
            return full_path
            break

        successors = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1)
        ]

        for move in successors:
            new_i, new_j = i + move[0], j + move[1]

            if is_valid(new_i, new_j) and not closed_list[new_i][new_j] and is_unblocked(grid, new_i, new_j):
                g_new = cell_details[i][j].g + (1.0 if abs(move[0]) + abs(move[1]) == 1 else 1.414)
                h_new = calculate_h_value(new_i, new_j, dest)
                f_new = g_new + h_new * grid[new_i][new_j]  # Consider terrain cost

                if cell_details[new_i][new_j].f == math.inf or cell_details[new_i][new_j].f > f_new:
                    open_list.put((f_new, (new_i, new_j)))
                    cell_details[new_i][new_j].f = f_new
                    cell_details[new_i][new_j].g = g_new
                    cell_details[new_i][new_j].h = h_new
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j

    if not found_dest:
        print("Failed to find the Destination Cell")

# Driver program to test above function
if __name__ == "__main__":
    grid = [
        [1, 1, 1, 1, 1, 0, 0, 1, 1, 1],
        [0, 0, 1, 1, 1, 1, 1, 1, 0, 1],
        [1, 1, 1, 0, 1, 0, 1, 0, 0, 1],
        [1, 0, 0, 0, 1, 1, 1, 0, 0, 1],
        [1, 1, 1, 1, 1, 0, 0, 1, 0, 0],
        [1, 0, 0, 0, 0, 0, 0, 1, 0, 1],
        [1, 0, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0, 1, 1, 1, 0, 1]
    ]

    src = (0, 0)
    dest = (4, 4)

    cell_details = a_star_search(grid, src, dest)
    print(cell_details)
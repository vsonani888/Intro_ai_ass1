import numpy as np
import matplotlib.pyplot as plt
from queue import PriorityQueue
import sys
import os
import time

from astar import repeated_backward_a_star, repeated_forward_a_star, adaptive_a_star

def load_maze(maze_filename):
    with open(maze_filename, 'r') as file:
        content = file.read().strip().split('MAZE')

    mazes = []

    #print("hi from load_maze", len(content[0]), " ", content)

    for i in range(1, len(content)):
        lines = content[i].strip().split('\n')
        #print("hi from loop", lines)
        maze = []
        for line in range(1, len(lines)):
            maze.append(list(lines[line]))
            #print(line, "lines")

        mazes.append(maze)

    # for maze in mazes:
    #     print(maze)
    #     print()

    return mazes


def forward_maze_runner(maze_filename, tie_break):
    mazes = load_maze(maze_filename)

    num_solvable = 0
    total_expanded = 0
    total_mazes = len(mazes)

    start_time = time.time()

    for maze in mazes:
        start = None
        goal = None

        rows = len(maze)
        cols = len(maze[0])

        for i in range(rows):
            for j in range(cols):
                if maze[i][j] == 'A': #its agent cell
                    start = (i, j)
                elif maze[i][j] == 'T': #its target cell
                    goal = (i, j)

        A_star_output = repeated_forward_a_star(maze, start, goal, tie_break) 

        path = A_star_output[0]
        expanded_nodes = A_star_output[1]

        if path: #path exists
            num_solvable += 1
            total_expanded += expanded_nodes

    end_time = time.time()
    total_time = end_time - start_time

    return num_solvable, total_expanded, total_time, total_mazes



if __name__ == "__main__":
    print("hi from main")

    maze_filename = 'generated_mazes_101.txt'
    total_tests_run = 50
    tests_passed = 0

    #part 2 forward a star smaller G vs larger G

    #smaller G
    tie_break = 'SMALLER_G'
    result_sg = forward_maze_runner(maze_filename, tie_break)

    solvable_sg = result_sg[0]
    expanded_sg = result_sg[1]
    time_sg = result_sg[2]
    total_mazes_sg = result_sg[3]

    #larger G
    tie_break = 'LARGER_G'
    result_lg = forward_maze_runner(maze_filename, tie_break)

    solvable_lg = result_lg[0]
    expanded_lg = result_lg[1]
    time_lg = result_lg[2]
    total_mazes_lg = result_lg[3]


    print("Part 2 results:")
    print(f"Smaller G block expansion: {expanded_sg} | total time: {time_sg:.4f}s")
    print(f"Larger G block expansion: {expanded_lg} | total time: {time_lg:.4f}s")

    #part 3 forward a star smaller G vs larger G

    



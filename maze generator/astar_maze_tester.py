import numpy as np
import matplotlib.pyplot as plt
from queue import PriorityQueue
import sys
import os

# Add the parent directory to the system path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'A star')))

# Import functions from astar.py
from astar import repeated_forward_a_star, repeated_backward_a_star, a_star_search, manhattan_distance, get_neighbors, reconstruct_path, adaptive_a_star

# Load mazes from file
def load_mazes(filename):
    with open(filename, 'r') as file:
        content = file.read().strip().split('MAZE')
    mazes = []
    for maze_text in content[1:]:
        lines = maze_text.strip().split('\n')[1:]
        maze = [list(line) for line in lines if line]
        mazes.append(maze)
    return mazes

# Visualization Function
def visualize_maze(grid, path=None, expanded=None, title="Maze Visualization"):
    size = len(grid)
    maze = np.zeros((size, size))
    for i in range(size):
        for j in range(size):
            if grid[i][j] == '#':
                maze[i][j] = 1  # Blocked
            elif grid[i][j] == 'A':
                maze[i][j] = 2  # Agent
            elif grid[i][j] == 'T':
                maze[i][j] = 3  # Target
            else:
                maze[i][j] = 0  # Unblocked
    plt.figure(figsize=(10, 10))
    cmap = plt.colormaps.get_cmap('viridis').resampled(4)

    plt.imshow(maze, cmap=cmap, origin='upper')
    plt.colorbar(ticks=[0, 1, 2, 3], label='Cell Type')
    plt.clim(-0.5, 3.5)
    if expanded:
        expanded_rows, expanded_cols = zip(*expanded)
        plt.scatter(expanded_cols, expanded_rows, color='orange', s=10, label='Expanded Nodes')
    if path:
        path_rows, path_cols = zip(*path)
        plt.plot(path_cols, path_rows, color='red', linewidth=2, label='Path')
    plt.legend()
    plt.title(title)
    plt.xticks([])
    plt.yticks([])
    plt.show()

# Test Repeated Backward A* on all mazes
def test_repeated_backward_mazes(filename):
    mazes = load_mazes(filename)
    solvable_mazes = 0
    total_expanded = 0

    for idx, maze in enumerate(mazes):
        start, goal = None, None
        for i in range(len(maze)):
            for j in range(len(maze[0])):
                if maze[i][j] == 'A':
                    start = (i, j)
                elif maze[i][j] == 'T':
                    goal = (i, j)

        if start and goal:
            path, expanded_nodes = repeated_backward_a_star(maze, start, goal)
            if path:
                solvable_mazes += 1
                total_expanded += expanded_nodes
                print(f"Maze {idx}: Solved in {expanded_nodes} expanded nodes.")
                # Visualize a few sample mazes (e.g., first 5)
                # if idx < 5:
                #     if isinstance(expanded_nodes, list):  # Only pass if it's a list
                #         visualize_maze(maze, path, expanded_nodes, title=f"Repeated Backward A* - Maze {idx}")
                #     else:
                #         visualize_maze(maze, path, None, title=f"Repeated Backward A* - Maze {idx}")

            else:
                print(f"Maze {idx}: No solution found.")
        else:
            print(f"Maze {idx}: Invalid start or goal positions.")

    print(f"\nTotal Solvable Mazes: {solvable_mazes}/{len(mazes)}")
    if solvable_mazes > 0:
        print(f"Average Nodes Expanded: {total_expanded / solvable_mazes:.2f}")

# Run Repeated Backward A* Tests on Generated Mazes
# test_repeated_backward_mazes('generated_mazes_101.txt')
# Test Adaptive A* on 50 mazes
def test_adaptive_a_star_on_mazes(filename):
    mazes = load_mazes(filename)
    solvable_mazes = 0
    total_expanded = 0

    for idx, maze in enumerate(mazes):
        start, goal = None, None
        for i in range(len(maze)):
            for j in range(len(maze[0])):
                if maze[i][j] == 'A':
                    start = (i, j)
                elif maze[i][j] == 'T':
                    goal = (i, j)

        if start and goal:
            path, expanded_nodes, heuristic = adaptive_a_star(maze, start, goal)
            if path:
                solvable_mazes += 1
                total_expanded += len(expanded_nodes)

                #print(f"Maze {idx}: Solved in {expanded_nodes} expanded nodes.")
                # Visualize a few sample mazes (e.g., first 5)
                # if idx < 5:
                #     visualize_maze(maze, path, title=f"Adaptive A* - Maze {idx}")
            else:
                print(f"Maze {idx}: No solution found.")
        else:
            print(f"Maze {idx}: Invalid start or goal positions.")

    print(f"\nTotal Solvable Mazes: {solvable_mazes}/{len(mazes)}")
    if solvable_mazes > 0:
        print(f"Average Nodes Expanded: {total_expanded / solvable_mazes:.2f}")

# Run Adaptive A* Tests on Generated Mazes
test_adaptive_a_star_on_mazes('generated_mazes_101.txt')
import numpy as np
import matplotlib.pyplot as plt
import os
import time
from queue import PriorityQueue
from astar import adaptive_a_star, get_neighbors, reconstruct_path, manhattan_distance

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

# Create directory for stepwise images
stepwise_output_dir = "stepwise_images"
os.makedirs(stepwise_output_dir, exist_ok=True)

# Visualization Function (Saves an image for each step)
def visualize_step(grid, path, expanded, step, filename):
    size = len(grid)
    maze = np.zeros((size, size))

    for i in range(size):
        for j in range(size):
            if grid[i][j] == '#':
                maze[i][j] = 1  # Blocked
            elif grid[i][j] == 'A':
                maze[i][j] = 2  # Start
            elif grid[i][j] == 'T':
                maze[i][j] = 3  # Target
            else:
                maze[i][j] = 0  # Free space

    plt.figure(figsize=(8, 8))
    cmap = plt.cm.viridis
    plt.imshow(maze, cmap=cmap, origin='upper')
    plt.colorbar(ticks=[0, 1, 2, 3], label='Cell Type')
    plt.clim(-0.5, 3.5)

    if expanded and isinstance(expanded, list):
        expanded_rows, expanded_cols = zip(*expanded)
        plt.scatter(expanded_cols, expanded_rows, color='orange', s=10, label='Expanded Nodes')

    if path and isinstance(path, list):
        path_rows, path_cols = zip(*path)
        plt.plot(path_cols, path_rows, color='red', linewidth=2, label='Path')

    plt.legend()
    plt.title(f"Step {step}: Path Progress")
    plt.xticks([])
    plt.yticks([])

    # Save the image for this step
    plt.savefig(os.path.join(stepwise_output_dir, filename))
    plt.close()

# Function to run A* and save step-by-step images
def test_stepwise_path(maze_index, filename):
    mazes = load_mazes(filename)
    
    if maze_index >= len(mazes):
        print(f"Maze index {maze_index} is out of range!")
        return

    maze = mazes[maze_index]
    start, goal = None, None
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            if maze[i][j] == 'A':
                start = (i, j)
            elif maze[i][j] == 'T':
                goal = (i, j)

    if not start or not goal:
        print(f"Maze {maze_index} has invalid start or goal positions.")
        return

    print(f"Running Repeated Forward A* on Maze {maze_index} with stepwise visualization...")

    # Initialize step counter
    step = 0
    path = []
    expanded_nodes = []

    # Run A* and capture every step
    grid_size = len(maze)
    open_list = PriorityQueue()
    g_score = {start: 0}
    f_score = {start: manhattan_distance(start, goal)}
    came_from = {}

    open_list.put((f_score[start], start))
    
    while not open_list.empty():
        _, current = open_list.get()
        expanded_nodes.append(current)

        # Save visualization for each step
        visualize_step(maze, path, expanded_nodes, step, filename=f"maze_{maze_index}_step_{step}.png")
        step += 1

        if current == goal:
            path = reconstruct_path(came_from, start, goal)
            print(f"Goal reached at {goal} in {step} steps.")
            break

        for neighbor in get_neighbors(current, grid_size):
            r, c = neighbor
            if maze[r][c] == '#':  
                continue  # Skip obstacles
            
            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + manhattan_distance(neighbor, goal)
                open_list.put((f_score[neighbor], neighbor))

    print(f"Stepwise images saved in '{stepwise_output_dir}/'.")
    return path

# Run the stepwise test on the first maze (index 0)
test_stepwise_path(maze_index=0, filename='generated_mazes_101.txt')

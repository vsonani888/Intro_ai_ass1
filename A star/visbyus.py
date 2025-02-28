import numpy as np
import matplotlib.pyplot as plt
import os
import time
from queue import PriorityQueue
from astarbyus import repeated_forward_a_star, get_neighbors, reconstruct_path, manhattan_distance

def load_mazes(filename):#just loading the mazes reading file
    with open(filename, 'r') as file:
        content = file.read().strip().split('MAZE')
    mazes = []
    for maze_text in content[1:]:
        lines = maze_text.strip().split('\n')[1:]
        maze = [list(line) for line in lines if line]
        mazes.append(maze)
    return mazes

stepwise_output_dir = "path_images"
os.makedirs(stepwise_output_dir, exist_ok=True)

def visualize_path(grid, path, expanded, step, filename):
    size = len(grid)
    maze = np.zeros((size, size))
    for i in range(size):#convert # A T into num so matplot can visualize
        for j in range(size):
            if grid[i][j] == '#':
                maze[i][j] = 1  # Blocked
            elif grid[i][j] == 'A':
                maze[i][j] = 2  # Start
            elif grid[i][j] == 'T':
                maze[i][j] = 3  # Target
            else:
                maze[i][j] = 0  # Free space
    plt.figure(figsize=(8, 8))#scale of grid
    cmap = plt.cm.viridis#color cell 
    plt.imshow(maze, cmap=cmap, origin='upper')#left up is start position
    plt.colorbar(ticks=[0, 1, 2, 3], label='Cell Type')#adds color
    plt.clim(-0.5, 3.5)#range for colors
    if expanded and isinstance(expanded, list):# makes the expanded nodes into two lists
        expanded_rows, expanded_cols = zip(*expanded)#row and col for plots of each cell in orange dot
        plt.scatter(expanded_cols, expanded_rows, color='orange', s=10, label='Expanded Nodes')
    if path and isinstance(path, list):#makes path into line shown in red
        path_rows, path_cols = zip(*path)
        plt.plot(path_cols, path_rows, color='red', linewidth=2, label='Path')
    plt.legend()
    plt.title(f"Step {step}: Path Progress")
    plt.xticks([])#get rid of ticks make it cleaner 
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
    maze = mazes[maze_index]#gets the maze index
    start, goal = None, None
    for i in range(len(maze)):
        for j in range(len(maze[0])):#find start and end position
            if maze[i][j] == 'A':
                start = (i, j)
            elif maze[i][j] == 'T':
                goal = (i, j)
    if not start or not goal:
        print(f"Maze {maze_index} has invalid start or goal positions.")
        return
    print(f"Running whatever A* on Maze {maze_index} with stepwise visualization...")
    # Initialize step counter
    step = 0#for num of steps
    path = []#store final path 
    expanded_nodes = []#keeps track of expanded nodes for visualization
    # Run A* and capture every step
    grid_size = len(maze)
    open_list = PriorityQueue()
    g_score = {start: 0}
    f_score = {start: manhattan_distance(start, goal)}
    came_from = {}
    open_list.put((f_score[start], start))
    while not open_list.empty():
        f_score_value, current = open_list.get()  # Unpack priority and node
        expanded_nodes.append(current)

        # Save visualization for each step
        visualize_path(maze, path, expanded_nodes, step, filename=f"maze_{maze_index}_step_{step}.png")
        step += 1

        if current == goal:
            path = reconstruct_path(came_from, start, goal)
            print(f"Goal reached at {goal} in {step} steps.")
            break

        for neighbor in get_neighbors(current, grid_size):  # Now correctly passing just (r, c)
            r, c = neighbor
            if maze[r][c] == '#':
                continue  # Skip obstacles

            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                tentative_f_score = tentative_g_score + manhattan_distance(neighbor, goal)

                if not isinstance(f_score, dict):  # Debugging step
                    raise TypeError(f"f_score should be a dictionary, but got {type(f_score)} instead.")

                f_score[neighbor] = tentative_f_score  # Ensure dictionary assignment
                open_list.put((tentative_f_score, neighbor))  # Ensure this tuple structure

    # while open_list:
    #     current = open_list.get()  # Extract the node with the lowest priority
    #     _, node = current  # Unpack priority and node
    #     expanded_nodes.append(node)
    #     # Save visualization for each step
    #     visualize_path(maze, path, expanded_nodes, step, filename=f"maze_{maze_index}_step_{step}.png")
    #     step += 1
    #     if current == goal:#found target position
    #         path = reconstruct_path(came_from, start, goal)#store path
    #         print(f"Goal reached at {goal} in {step} steps.")
    #         break
    #     for neighbor in get_neighbors(current, grid_size):
    #         r, c = neighbor
    #         if maze[r][c] == '#':  
    #             continue  # Skip obstacles
    #         tentative_g_score = g_score[current] + 1
    #         if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
    #             came_from[neighbor] = current#updating the g score if shorter path
    #             g_score[neighbor] = tentative_g_score
    #             f_score[neighbor] = tentative_g_score + manhattan_distance(neighbor, goal)#update f score estimated total cost
    #             open_list.put((f_score[neighbor], neighbor))
    print(f"Stepwise images saved in '{stepwise_output_dir}/'.")
    return path
test_stepwise_path(maze_index=0, filename='generated_mazes_101.txt')

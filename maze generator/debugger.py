import matplotlib.pyplot as plt
import numpy as np

# Function to parse maze from text file
def parse_mazes(file_path):
    with open(file_path, 'r') as file:
        content = file.read().strip().split('MAZE')
    mazes = []
    for maze_text in content[1:]:
        lines = maze_text.strip().split('\n')[1:]  # Skip the maze number line
        maze = []
        for line in lines:
            if line.strip() and not line.startswith('-'):
                maze.append(list(line.strip().strip('|')))
        mazes.append(maze)
    return mazes

# Visualization function to display maze and detected start/goal
def visualize_maze_debug(grid, start=None, goal=None, title="Maze Debug Visualization"):
    size = len(grid)
    maze = np.zeros((size, size))
    for i in range(size):
        for j in range(size):
            if grid[i][j] in ['#', '0']:
                maze[i][j] = 1  # Blocked
            elif (i, j) == start:
                maze[i][j] = 2  # Start
            elif (i, j) == goal:
                maze[i][j] = 3  # Goal
            else:
                maze[i][j] = 0  # Unblocked
    plt.figure(figsize=(8, 8))
    cmap = plt.cm.get_cmap('viridis', 4)
    plt.imshow(maze, cmap=cmap, origin='upper')
    plt.colorbar(ticks=[0, 1, 2, 3], label='Cell Type')
    plt.clim(-0.5, 3.5)
    plt.title(title)
    plt.xticks([])
    plt.yticks([])
    plt.show()

# Debugging function to print and visualize mazes
def debug_mazes(file_path):
    mazes = parse_mazes(file_path)
    for idx, maze in enumerate(mazes):
        print(f"--- Maze {idx} ---")
        start, goal = None, None
        for i in range(len(maze)):
            for j in range(len(maze[0])):
                if maze[i][j] in ['S', 'A']:
                    start = (i, j)
                elif maze[i][j] in ['E', 'T']:
                    goal = (i, j)
        # Print detected start and goal
        print(f"Start Position: {start}, Goal Position: {goal}")
        # Print maze grid for manual inspection
        for row in maze:
            print(''.join(row))
        # Visualize the maze with detected positions
        visualize_maze_debug(maze, start, goal, title=f"Maze {idx} Debug Visualization")

# Run debug on a maze file
debug_mazes('maze_test.txt')

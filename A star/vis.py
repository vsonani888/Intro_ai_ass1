import matplotlib.pyplot as plt
import numpy as np

def visualize_maze(grid, path=None, title="Maze Visualization"):
    """
    Visualizes the maze using matplotlib.
    
    Args:
        grid (list[list[str]]): 2D maze grid with '_', '#', 'A', 'T'.
        path (list[tuple]): List of (row, col) tuples representing the found path.
        title (str): Title of the plot.
    """
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
    
    plt.figure(figsize=(8, 8))
    cmap = plt.cm.get_cmap('viridis', 4)  # 4 colors: Unblocked, Blocked, Agent, Target
    plt.imshow(maze, cmap=cmap, origin='upper')
    plt.colorbar(ticks=[0, 1, 2, 3], label='Cell Type')
    plt.clim(-0.5, 3.5)
    
    if path:
        path_rows, path_cols = zip(*path)
        plt.plot(path_cols, path_rows, color='red', linewidth=2, label='Path')
        plt.legend()
    
    plt.title(title)
    plt.xticks([])
    plt.yticks([])
    plt.show()

# Example Usage
grid_example = [
    ['_', '_', '_', '_', '#'],
    ['_', '#', '_', '#', '_'],
    ['_', '#', 'A', '_', '_'],
    ['_', '#', '_', '#', '_'],
    ['#', '_', '_', '_', 'T']
]

# Example path for testing
example_path = [(2, 2), (2, 3), (3, 3), (4, 3), (4, 4)]

# Visualize the maze with a path
visualize_maze(grid_example, path=example_path, title="Example Maze with Path")

import numpy as np
import matplotlib.pyplot as plt
from repeatedA import repeatedForwardAStarB
def visualize_grid(grid, path, start, goal):
    """
    Visualizes the grid with the path from start to goal.

    Arguments:
    grid -- 2D numpy array representing the grid (1=unblocked, 2=blocked)
    path -- List of tuples representing the path taken
    start -- Tuple (row, col) representing the starting position
    goal -- Tuple (row, col) representing the goal position
    """

    plt.figure(figsize=(6, 6))
    cmap = plt.get_cmap("gray_r")  # Use grayscale for the grid

    # Display the grid
    plt.imshow(grid, cmap=cmap, origin="upper")

    # Mark the path in blue
    path_x, path_y = zip(*path)  # Extract x and y coordinates from path
    plt.plot(path_y, path_x, marker="o", color="blue", markersize=5, label="Path")

    # Mark start and goal points
    plt.scatter(start[1], start[0], marker="s", color="green", s=100, label="Start")  # Start (Green)
    plt.scatter(goal[1], goal[0], marker="s", color="red", s=100, label="Goal")  # Goal (Red)

    # Labels and legend
    plt.xticks(range(grid.shape[1]))
    plt.yticks(range(grid.shape[0]))
    plt.grid(visible=True, color="black", linewidth=0.5)
    plt.legend()
    plt.title("Repeated Forward A* Path Visualization")
    plt.show()

# Example Usage
if __name__ == "__main__":
    grid_size = 10
    grid = np.ones((grid_size, grid_size))  # Initialize all cells as unblocked

    # Sample obstacles (same as used in the algorithm)
    obstacles = [(3, 3), (3, 4), (4, 4), (5, 4)]
    for obs in obstacles:
        grid[obs] = 2  # Mark blocked cells as 2

    start = (0, 0)
    goal = (9, 9)

    # Run the pathfinding algorithm
    path, expanded_nodes = repeatedForwardAStarB(grid, start, goal)

    # Visualize the path
    if path:
        visualize_grid(grid, path, start, goal)
    else:
        print("No Path Found")

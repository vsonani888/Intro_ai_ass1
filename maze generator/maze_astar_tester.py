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

# A* Implementation
def manhattan_distance(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def get_neighbors(position, grid_size):
    r, c = position
    neighbors = []
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nr, nc = r + dr, c + dc
        if 0 <= nr < grid_size and 0 <= nc < grid_size:
            neighbors.append((nr, nc))
    return neighbors

def a_star_search(grid, start, goal):
    from queue import PriorityQueue
    grid_size = len(grid)
    open_list = PriorityQueue()
    open_list.put((0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: manhattan_distance(start, goal)}
    
    while not open_list.empty():
        _, current = open_list.get()
        if current == goal:
            return reconstruct_path(came_from, start, goal)
        
        for neighbor in get_neighbors(current, grid_size):
            if grid[neighbor[0]][neighbor[1]] in ['#', '0']:  # Skip blocked cells
                continue
            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + manhattan_distance(neighbor, goal)
                open_list.put((f_score[neighbor], neighbor))
    return None

def reconstruct_path(came_from, start, goal):
    path = [goal]
    current = goal
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

# Visualization function
def visualize_maze(grid, path=None, title="Maze Visualization"):
    size = len(grid)
    maze = np.zeros((size, size))
    for i in range(size):
        for j in range(size):
            if grid[i][j] in ['#', '0']:
                maze[i][j] = 1  # Blocked
            elif grid[i][j] in ['S', 'A']:
                maze[i][j] = 2  # Agent
            elif grid[i][j] in ['E', 'T']:
                maze[i][j] = 3  # Target
            else:
                maze[i][j] = 0  # Unblocked
    plt.figure(figsize=(8, 8))
    cmap = plt.cm.get_cmap('viridis', 4)
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

# Testing function to run A* on all mazes
def test_mazes(file_path):
    mazes = parse_mazes(file_path)
    for idx, maze in enumerate(mazes):
        start, goal = None, None
        for i in range(len(maze)):
            for j in range(len(maze[0])):
                if maze[i][j] in ['S', 'A']:
                    start = (i, j)
                elif maze[i][j] in ['E', 'T']:
                    goal = (i, j)
        if start and goal:
            path = a_star_search(maze, start, goal)
            if path:
                print(f"Path found for Maze {idx}")
                visualize_maze(maze, path, title=f"Maze {idx} with A* Path")
            else:
                print(f"No path found for Maze {idx}")
        else:
            print(f"Start or goal not found for Maze {idx}")

# Run the test on maze_test.txt
test_mazes('maze_test.txt')

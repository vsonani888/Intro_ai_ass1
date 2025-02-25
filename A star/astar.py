import numpy as np
import matplotlib.pyplot as plt
from queue import PriorityQueue

# Manhattan Distance Heuristic
def manhattan_distance(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# Neighbor Function for Gridworld
def get_neighbors(position, grid_size):
    r, c = position
    neighbors = []
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nr, nc = r + dr, c + dc
        if 0 <= nr < grid_size and 0 <= nc < grid_size:
            neighbors.append((nr, nc))
    return neighbors

# A* Search Algorithm with Debug Info
# def a_star_search(grid, start, goal):
#     grid_size = len(grid)
#     open_list = PriorityQueue()
#     open_list.put((0, start))
#     came_from = {}
#     g_score = {start: 0}
#     f_score = {start: manhattan_distance(start, goal)}
#     expanded_nodes = []
    
#     while not open_list.empty():
#         _, current = open_list.get()
#         expanded_nodes.append(current)
#         #print(f"Expanding node: {current}")
#         if current == goal:
#             path = reconstruct_path(came_from, start, goal)
#             return path, expanded_nodes
        
#         for neighbor in get_neighbors(current, grid_size):
#             cell_value = grid[neighbor[0]][neighbor[1]]
#             if cell_value == '#':
#                 #print(f"Skipping blocked cell: {neighbor}")
#                 continue  # Skip blocked cells
#             tentative_g_score = g_score[current] + 1
#             if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
#                 came_from[neighbor] = current
#                 g_score[neighbor] = tentative_g_score
#                 f_score[neighbor] = tentative_g_score + manhattan_distance(neighbor, goal)
#                 open_list.put((f_score[neighbor], neighbor))
#     return None, expanded_nodes  # No path found


def a_star_search(grid, start, goal, tie_break='LARGER_G'):
    grid_size = len(grid)
    open_list = PriorityQueue()

    g_score = {start: 0}
    f_score = {start: manhattan_distance(start, goal)}
    came_from = {}
    
    # Tie-breaking logic for A* priority queue
    if tie_break == 'LARGER_G':
        open_list.put((f_score[start], -g_score[start], start))
    else:
        open_list.put((f_score[start], g_score[start], start))

    expanded_nodes = []

    while not open_list.empty():
        _, _, current = open_list.get()
        expanded_nodes.append(current)

        if current == goal:
            path = reconstruct_path(came_from, start, goal)
            return path, len(expanded_nodes)  # Return the count of expanded nodes

        for neighbor in get_neighbors(current, grid_size):
            r, c = neighbor
            if grid[r][c] == '#':  
                continue

            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + manhattan_distance(neighbor, goal)

                if tie_break == 'LARGER_G':
                    open_list.put((f_score[neighbor], -g_score[neighbor], neighbor))
                else:
                    open_list.put((f_score[neighbor], g_score[neighbor], neighbor))

    return None, len(expanded_nodes)  # Return the count



# Repeated Forward A* Algorithm
# def repeated_forward_a_star(true_grid, start, goal):
#     grid_size = len(true_grid)
#     knowledge_grid = [['_' for _ in range(grid_size)] for _ in range(grid_size)]
#     current_position = start
#     total_expanded = 0
#     full_path = []

#     while current_position != goal:
#         # Plan path using current knowledge
#         path, _ = a_star_search(knowledge_grid, current_position, goal)
#         if path is None:
#             print("No path found with current knowledge.")
#             return None, total_expanded
        
#         # Follow the path until an obstacle is encountered or the goal is reached
#         for step in path[1:]:
#             full_path.append(step)
#             total_expanded += 1
#             if true_grid[step[0]][step[1]] == '#':  # Obstacle discovered
#                 knowledge_grid[step[0]][step[1]] = '#'  # Update knowledge
#                 break  # Replan
#             else:
#                 current_position = step
#                 if current_position == goal:
#                     print(f"Goal reached at {current_position}")
#                     return full_path, total_expanded
    
#     return full_path, total_expanded

def repeated_forward_a_star(true_grid, start, goal, tie_break='LARGER_G'):
    """
    Repeated Forward A* with tie-breaking support.

    :param true_grid: The actual/grid world with obstacles (#) and free spaces (_).
    :param start: (row, col) start position.
    :param goal: (row, col) goal position.
    :param tie_break: 'LARGER_G' or 'SMALLER_G' to influence which node is expanded first when f-values tie.
    :return: (full_path, total_expanded)
       - full_path: List of (row, col) cells from start to goal (or partial if blocked).
       - total_expanded: Count of expansions (or steps, depending on your definition).
    """

    grid_size = len(true_grid)
    # Initialize the knowledge grid with unknown cells marked '_'
    knowledge_grid = [['_' for _ in range(grid_size)] for _ in range(grid_size)]

    current_position = start
    total_expanded = 0
    full_path = []

    while current_position != goal:
        # 1) Plan path using the agent's current knowledge + tie-break
        path, expanded_in_search = a_star_search(
            knowledge_grid,            # The "known" world
            current_position,          # Current position
            goal,                      # Goal position
            tie_break=tie_break        # Pass tie-break strategy to A* 
        )

        # If no path is found, the agent is stuck
        if path is None:
            print("No path found with current knowledge.")
            return None, total_expanded

        # Optionally accumulate the expansions performed during the A* search
        total_expanded += expanded_in_search

        # 2) Follow the path until an obstacle is encountered or the goal is reached
        for step in path[1:]:  # path[0] is the current position
            full_path.append(step)
            
            # If we discover an obstacle in the *true* grid (not known yet)
            if true_grid[step[0]][step[1]] == '#':
                # Update knowledge
                knowledge_grid[step[0]][step[1]] = '#'
                # Break so we can plan again
                break
            else:
                # It's a free cell; move the agent
                current_position = step
                # Check if we've reached the goal
                if current_position == goal:
                    print(f"Goal reached at {current_position}")
                    return full_path, total_expanded

    # If the while loop ends normally (somehow), return the path
    return full_path, total_expanded


def repeated_backward_a_star(true_grid, start, goal):
    grid_size = len(true_grid)
    knowledge_grid = [['_' for _ in range(grid_size)] for _ in range(grid_size)]
    current_position = goal  # Start from the goal
    total_expanded = 0
    full_path = []

    while current_position != start:
        # Plan path from the current position to the start (search backward)
        path, _ = a_star_search(knowledge_grid, current_position, start)
        if path is None:
            print("No path found with current knowledge.")
            return None, total_expanded
        
        # Follow the path until an obstacle is encountered or the start is reached
        for step in path[1:]:
            full_path.append(step)
            total_expanded += 1
            if true_grid[step[0]][step[1]] == '#':  # Obstacle discovered
                knowledge_grid[step[0]][step[1]] = '#'  # Update knowledge
                break  # Replan
            else:
                current_position = step
                if current_position == start:
                    print(f"Start reached at {current_position}")
                    full_path.reverse()  # Reverse to get path from start to goal
                    return full_path, total_expanded

    full_path.reverse()  # Reverse to ensure proper direction
    return full_path, expanded_nodes 

def adaptive_a_star(true_grid, start, goal):
    grid_size = len(true_grid)
    knowledge_grid = [['_' for _ in range(grid_size)] for _ in range(grid_size)]
    current_position = start
    total_expanded = 0
    full_path = []

    while current_position != goal:
        # Plan path using current knowledge
        path, _ = a_star_search(knowledge_grid, current_position, goal)
        if path is None:
            print("No path found with current knowledge.")
            return None, total_expanded
        
        # Follow the path until an obstacle is encountered or the goal is reached
        for step in path[1:]:
            full_path.append(step)
            total_expanded += 1
            if true_grid[step[0]][step[1]] == '#':  # Obstacle discovered
                knowledge_grid[step[0]][step[1]] = '#'  # Update knowledge
                break  # Replan
            else:
                current_position = step
                if current_position == goal:
                    print(f"Goal reached at {current_position}")
                    return full_path, total_expanded
    
    return full_path, total_expanded

# Path Reconstruction
def reconstruct_path(came_from, start, goal):
    path = [goal]
    current = goal
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

# Visualize Maze Function
def visualize_maze_debug(grid, path=None, expanded=None, title="Maze Visualization with Debug"):
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
    cmap = plt.cm.get_cmap('viridis', 4)
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

# Test Function for A* on Generated Maze
def test_astar_on_maze_debug():
    grid_example = [
        ['_', '_', '_', '_', '#'],
        ['_', '#', '_', '#', '_'],
        ['_', '#', 'A', '_', '_'],
        ['_', '#', '_', '#', '_'],
        ['#', '_', '_', '_', 'T']
    ]
    
    # Find agent and target positions
    start, goal = None, None
    for i in range(len(grid_example)):
        for j in range(len(grid_example[0])):
            if grid_example[i][j] == 'A':
                start = (i, j)
            elif grid_example[i][j] == 'T':
                goal = (i, j)
    
    # Run A* Algorithm
    #path, expanded_nodes = a_star_search(grid_example, start, goal)
    path, expanded_nodes = repeated_forward_a_star(maze, start, goal)

    # Visualize the Maze with Path and Expanded Nodes
    #visualize_maze_debug(grid_example, path, expanded_nodes, title="A* Debug Visualization on Example Maze")

# Run the Debug Test
#test_astar_on_maze_debug()

# Test Function for Repeated Forward A* on Generated Maze
def test_repeated_forward_astar():
    grid_example = [
        ['_', '_', '_', '_', '#'],
        ['_', '#', '_', '#', '_'],
        ['_', '#', 'A', '_', '_'],
        ['_', '#', '_', '#', '_'],
        ['#', '_', '_', '_', 'T']
    ]
    
    # Find agent and target positions
    start, goal = None, None
    for i in range(len(grid_example)):
        for j in range(len(grid_example[0])):
            if grid_example[i][j] == 'A':
                start = (i, j)
            elif grid_example[i][j] == 'T':
                goal = (i, j)
    
    # Run Repeated Forward A* Algorithm
    path, total_expanded = repeated_forward_a_star(grid_example, start, goal)
    print(f"Path ends at: {path[-1]}")
    print(f"Goal position: {goal}")
    # Visualize the Maze with Path and Expanded Nodes
    if path:
        print(f"Repeated Forward A* found a path with {total_expanded} nodes expanded.")
        #visualize_maze_debug(grid_example, path, None, title="Repeated Forward A* Debug Visualization")
    else:
        print("Repeated Forward A* could not find a path.")
#test_repeated_forward_astar()

# Test Function for Repeated Backward A* on Generated Maze
def test_repeated_backward_astar():
    grid_example = [
        ['_', '_', '_', '_', '#'],
        ['_', '#', '_', '#', '_'],
        ['_', '#', 'A', '_', '_'],
        ['_', '#', '_', '#', '_'],
        ['#', '_', '_', '_', 'T']
    ]
    
    # Find agent and target positions
    start, goal = None, None
    for i in range(len(grid_example)):
        for j in range(len(grid_example[0])):
            if grid_example[i][j] == 'A':
                start = (i, j)
            elif grid_example[i][j] == 'T':
                goal = (i, j)
    
    # Run Repeated Backward A* Algorithm
    path, total_expanded = repeated_backward_a_star(grid_example, start, goal)
    print(f"Path ends at: {path[-1]}")
    print(f"Start position: {start}")
    # Visualize the Maze with Path and Expanded Nodes
    if path:
        print(f"Repeated Backward A* found a path with {total_expanded} nodes expanded.")
        #visualize_maze_debug(grid_example, path, None, title="Repeated Backward A* Debug Visualization")
    else:
        print("Repeated Backward A* could not find a path.")

#test_repeated_backward_astar()
#test_repeated_forward_astar()
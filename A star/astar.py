import numpy as np
import matplotlib.pyplot as plt
#from queue import PriorityQueue
from binaryheapbyus import BinaryHeap
import random

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
    open_list = BinaryHeap()

    g_score = {start: 0}
    f_score = {start: manhattan_distance(start, goal)}
    came_from = {}
    
    # Tie-breaking logic for A* priority queue
    if tie_break == 'LARGER_G':
        open_list.push((f_score[start], -g_score[start], start))
    else:
        open_list.push((f_score[start], g_score[start], start))

    expanded_nodes = []

    while not open_list.is_empty():
        _, _, current = open_list.pop()
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
                    open_list.push((f_score[neighbor], -g_score[neighbor], neighbor))
                else:
                    open_list.push((f_score[neighbor], g_score[neighbor], neighbor))

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

def repeated_forward_a_star(grid, start, goal, tie_break='LARGER_G'):
    grid_size = len(grid)
    open_list = BinaryHeap()
    
    g_score = {start: 0}
    f_score = {start: manhattan_distance(start, goal)}
    came_from = {}
    
    if tie_break == 'LARGER_G':
        open_list.push((f_score[start], -g_score[start], start))
    else:
        open_list.push((f_score[start], g_score[start], start))
    
    expanded_nodes = []
    
    while not open_list.is_empty():
        _, _, current = open_list.pop()
        expanded_nodes.append(current)
        
        if current == goal:
            path = reconstruct_path(came_from, start, goal)
            return path, len(expanded_nodes)
        
        for neighbor in get_neighbors(current, grid_size):
            r, c = neighbor
            if grid[r][c] == '#':  
                continue  # Skip obstacles
            
            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + manhattan_distance(neighbor, goal)
                
                if tie_break == 'LARGER_G':
                    open_list.push((f_score[neighbor], -g_score[neighbor], neighbor))
                else:
                    open_list.push((f_score[neighbor], g_score[neighbor], neighbor))
    
    return None, len(expanded_nodes)



def repeated_backward_a_star(grid, start, goal, tie_break='LARGER_G'):
    """
    Perform a single backward A* search in a grid from 'goal' to 'start'.
    - grid: 2D list representing the grid; '#' represents blocked cells.
    - start: The agent's position (row, col).
    - goal:  The target position (row, col).
    - tie_break: 'LARGER_G' or 'SMALLER_G' indicates how to break ties.
    
    Returns:
      path, expanded_count
    where 'path' is a list of (row, col) from 'start' to 'goal' (in forward direction),
    and expanded_count is the number of unique cells expanded.
    """
    
    grid_size = len(grid)
    
    # For the BACKWARD search, we treat 'goal' as the "start node" of the search
    # and 'start' as our "goal node" in the search sense.
    # We'll compute f(n) = g(n) + h(n), where h(n) = manhattan_distance(n, start),
    # because the search is going from 'goal' -> 'start'.
    
    open_list = BinaryHeap()
    came_from = {}
    expanded_nodes = []
    
    # g_score[node] = best distance found so far from 'goal' (our search-root) to node
    g_score = {goal: 0}
    # f_score[node] = g_score[node] + h(node), where h(node) = distance to 'start'
    f_score = {goal: manhattan_distance(goal, start)}
    
    # Push the 'goal' node onto open_list
    if tie_break == 'LARGER_G':
        open_list.push((f_score[goal], -g_score[goal], goal))
    else:
        open_list.push((f_score[goal], g_score[goal], goal))
    
    # Standard A* loop, but we check "if current == start" as our termination
    while not open_list.is_empty():
        _, _, current = open_list.pop()
        
        # Keep track of expanded nodes
        if current not in expanded_nodes:
            expanded_nodes.append(current)
        
        # If we've reached the "start" in the backward sense, we are done.
        if current == start:
            # Reconstruct the path from 'goal' -> 'start', but then reverse it
            backward_path = reconstruct_path(came_from, goal, start)
            forward_path  = list(reversed(backward_path))
            return forward_path, len(expanded_nodes)
        
        # Expand neighbors
        for neighbor in get_neighbors(current, grid_size):
            r, c = neighbor
            if grid[r][c] == '#':
                continue  # Skip blocked
            
            # cost is 1 if neighbor is passable
            tentative_g_score = g_score[current] + 1
            
            if (neighbor not in g_score) or (tentative_g_score < g_score[neighbor]):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                # Our heuristic is distance to 'start' (since we're doing backward search)
                f_score[neighbor] = tentative_g_score + manhattan_distance(neighbor, start)
                
                if tie_break == 'LARGER_G':
                    open_list.push((f_score[neighbor], -g_score[neighbor], neighbor))
                else:
                    open_list.push((f_score[neighbor], g_score[neighbor], neighbor))
    
    # If we exhaust the open_list without reaching 'start', no path was found
    return None, len(expanded_nodes)



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
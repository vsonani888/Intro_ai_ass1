import numpy as np
import matplotlib.pyplot as plt
#from queue import PriorityQueue
from binaryheapbyus import BinaryHeap
import random
import os

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


# def a_star_search(grid, start, goal, tie_break='LARGER_G'):
#     grid_size = len(grid)
#     open_list = BinaryHeap()

#     g_score = {start: 0}
#     f_score = {start: manhattan_distance(start, goal)}
#     came_from = {}
    
#     # Tie-breaking logic for A* priority queue
#     if tie_break == 'LARGER_G':
#         open_list.push((f_score[start], -g_score[start], start))
#     else:
#         open_list.push((f_score[start], g_score[start], start))

#     expanded_nodes = []

#     while not open_list.is_empty():
#         _, _, current = open_list.pop()
#         expanded_nodes.append(current)

#         if current == goal:
#             path = reconstruct_path(came_from, start, goal)
#             return path, len(expanded_nodes)  # Return the count of expanded nodes

#         for neighbor in get_neighbors(current, grid_size):
#             r, c = neighbor
#             if grid[r][c] == '#':  
#                 continue

#             tentative_g_score = g_score[current] + 1
#             if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
#                 came_from[neighbor] = current
#                 g_score[neighbor] = tentative_g_score
#                 f_score[neighbor] = tentative_g_score + manhattan_distance(neighbor, goal)

#                 if tie_break == 'LARGER_G':
#                     open_list.push((f_score[neighbor], -g_score[neighbor], neighbor))
#                 else:
#                     open_list.push((f_score[neighbor], g_score[neighbor], neighbor))

#     return None, len(expanded_nodes)  # Return the count



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
        open_list.push((f_score[start], -g_score[start], random.random(), start))
    else:
        open_list.push((f_score[start], g_score[start], random.random(), start))
    
    expanded_nodes = []
    
    while not open_list.is_empty():
        _, _, _, current = open_list.pop()
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
                    open_list.push((f_score[neighbor], -g_score[neighbor], random.random(), neighbor))
                else:
                    open_list.push((f_score[neighbor], g_score[neighbor], random.random(), neighbor))
    
    return None, len(expanded_nodes)
    



def repeated_backward_a_star(grid, start, goal, tie_break='LARGER_G'):
    """Run one backward A* search from 'goal' to 'start' on 'grid',
       return path in forward direction and number of expanded cells."""

    grid_size = len(grid)
    open_list = BinaryHeap()
    came_from = {}
    expanded_nodes = []

    # Initialize g- and f-scores at the goal (since we search backwards)
    g_score = {goal: 0}
    f_score = {goal: manhattan_distance(goal, start)}

    # Push into the open list with the tie-break ordering
    if tie_break == 'LARGER_G':
        open_list.push((f_score[goal], -g_score[goal], random.random(), goal))
    else:
        open_list.push((f_score[goal], g_score[goal], random.random(), goal))

    # Main A* loop
    while not open_list.is_empty():
        _, _, _, current = open_list.pop()
        expanded_nodes.append(current)

        # If we've reached the 'start' (in the backward sense),
        # reconstruct the path from goal->start and reverse it.
        if current == start:
            backward_path = reconstruct_path(came_from, goal, start)
            forward_path  = list(reversed(backward_path))
            return forward_path, len(expanded_nodes)

        # Explore neighbors (i.e., predecessors when searching backward)
        for neighbor in get_neighbors(current, grid_size):
            r, c = neighbor
            # Skip obstacles
            if grid[r][c] == '#':
                continue

            tentative_g_score = g_score[current] + 1

            # Standard A* relaxation
            if (neighbor not in g_score) or (tentative_g_score < g_score[neighbor]):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + manhattan_distance(neighbor, start)

                if tie_break == 'LARGER_G':
                    open_list.push((f_score[neighbor], -g_score[neighbor], random.random(), neighbor))
                else:
                    open_list.push((f_score[neighbor], g_score[neighbor], random.random(), neighbor))

    # No path found
    return None, len(expanded_nodes)




def adaptive_a_star(grid, start, goal, h_score=None):
    """
    Performs one A* search from 'start' to 'goal' using an adaptive heuristic,
    then updates the heuristic for all expanded states and returns:
      (path, num_expanded, updated_h_score).

    :param grid: 2D list representing the environment ('.' for free, '#' for blocked).
    :param start: (row, col)
    :param goal:  (row, col)
    :param h_score: dict from state -> float (the stored heuristic for that state).
                    If None, we'll initialize a fresh dict.
    """
    # If no h_score dict was given, initialize an empty one
    if h_score is None:
        h_score = {}

    # Initialize open_list with your BinaryHeap
    open_list = BinaryHeap()

    # Use the stored heuristic if present; else default to Manhattan
    start_h = h_score.get(start, manhattan_distance(start, goal))

    # Push (f_value, state) into the BinaryHeap
    open_list.push((start_h, random.random(), start))

    came_from = {}
    g_score = {start: 0}  # cost from start to each discovered state
    expanded_nodes = []
    closed_set = set()    # to avoid re-expanding states

    while not open_list.is_empty():
        # Pop the entry with the smallest (f_value, ...)
        _, _, current = open_list.pop()

        if current in closed_set:
            continue

        closed_set.add(current)
        expanded_nodes.append(current)

        # If we've reached the goal, reconstruct path & update heuristics
        if current == goal:
            path = reconstruct_path(came_from, start, goal)
            update_adaptive_heuristic(h_score, g_score, expanded_nodes, goal)
            return path, len(expanded_nodes), h_score

        # Explore neighbors
        current_g = g_score[current]
        for neighbor in get_neighbors(current, len(grid)):
            # Skip blocked cells
            if grid[neighbor[0]][neighbor[1]] == '#':
                continue

            tentative_g = current_g + 1
            # If neighbor is new or we found a cheaper path
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                # Retrieve any updated heuristic or fall back to Manhattan
                neighbor_h = h_score.get(neighbor, manhattan_distance(neighbor, goal))
                f_score = tentative_g + neighbor_h

                # Push into our BinaryHeap
                open_list.push((f_score, random.random(), neighbor))

    # If we exhaust the open list without finding the goal, do final updates
    update_adaptive_heuristic(h_score, g_score, expanded_nodes, goal)
    return [], len(expanded_nodes), h_score



# Update Adaptive Heuristic Function
def update_adaptive_heuristic(h_score, g_score, expanded_states, goal):
    """
    For each expanded state s, update the heuristic:
        h_score[s] := g_score[goal] - g_score[s]
    (only if s is actually reachable and in g_score).
    """
    if goal not in g_score:
        return  # We never reached the goal; can't do updates.

    goal_g = g_score[goal]
    for s in expanded_states:
        if s in g_score:  # only update if we actually set a g-score for s
            h_score[s] = max(h_score.get(s, 0), goal_g - g_score[s])
    # The above "max(...)" ensures that if you already had a bigger heuristic 
    # from previous searches, you keep it.  (Optional but often good for consistency.)

# Path Reconstruction
def reconstruct_path(came_from, start, goal):
    path = [goal]
    current = goal
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

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
# def test_astar_on_maze_debug():
#     grid_example = [
#         ['_', '_', '_', '_', '#'],
#         ['_', '#', '_', '#', '_'],
#         ['_', '#', 'A', '_', '_'],
#         ['_', '#', '_', '#', '_'],
#         ['#', '_', '_', '_', 'T']
#     ]
    
#     # Find agent and target positions
#     start, goal = None, None
#     for i in range(len(grid_example)):
#         for j in range(len(grid_example[0])):
#             if grid_example[i][j] == 'A':
#                 start = (i, j)
#             elif grid_example[i][j] == 'T':
#                 goal = (i, j)
    
#     # Run A* Algorithm
#     #path, expanded_nodes = a_star_search(grid_example, start, goal)
#     path, expanded_nodes = repeated_forward_a_star(grid_example, start, goal)

#     # Visualize the Maze with Path and Expanded Nodes
#     #visualize_maze_debug(grid_example, path, expanded_nodes, title="A* Debug Visualization on Example Maze")

# # Run the Debug Test
#test_astar_on_maze_debug()

# Test Function for Repeated Forward A* on Generated Maze
def test_repeated_forward_astar():
    asci_grid = """
    #__##_###_#_#_#____#_____#____#_________#_###__#___##_#______#_#_#_____##____#______#_#_#______#_#__#
    _#__#____#_______##_______#_______##_________#______#__#____#____#________#____#_#_#___##__##__#___#_
    __#_#___#_#_______##_____#___#____##____________#___#_____#___#_#______#______#_____###__#_##_#_____#
    _#_##__#_#___#___#__##_##_#____________#_##_##__#__#_###_#_#____#_#__#____#__#___#__#_###_#______#__#
    ___##_#__####__#______#__##___#_#_____#_##__#__##_#_#___#_#__#______##_________#__#_________####_##__
    ##__#_________#_______#___#____#__#_##__#_#__###____#___##_##______###_###_#___#___#_#___#___#__#_##_
    ____#___#__#_______#______#_##___#__##___#_##___##______#__#_________#____________#__#_#___#_______#_
    ___##___##_#_#____#_##_#__#__#___##__##__#_#___#___#__##__________#_#_#__#______##___#_________#_____
    _##________###_____#_____#___#_#__#__##__#___#_#______#___##____#______#_#____________#___##____#_#__
    __#___#____###_#_#_____##_#_##___##__#________##_##__##_________#___#_#_##_#_#___##_#____#___#___##_#
    #_________#__#_____##____#________##___#__#_#_##__#___#______#_________#______#_#_####______#________
    #_#____#__________#_________###___##_#____##_______#____#__#__##___#_##__##____#_#_#___##___#______#_
    _#__#____________#___#____#__________#________#####_______###__#_#_#______#_##__#_#_#_____#_#_____#_#
    _____#__#___#_#____###_#__#__##__##_____#______#_____________#_____##____#___#__###_______________#__
    _____#____#_____#__#___#_##________#__#_______#____#______#_#_________#__####_#______#__##__#___#___#
    #_#__#_####_#___#__#___#__##__#_#___#___#_##_#_________#_###________#__#__##__#__#__#_#_#_____#_____#
    _#_#_##____#____#_##_____#__________#_____#####__##___#_____#__##___##_####__#__#__#__#_______#_____#
    __#__#_____#__#__#_____#__#_####_##__##___________#__#____##_#__##_##______#_#_#____#__#_#_________#_
    _##__#_#_#____#__##_##___________#_##_#_#______##______#__#_#__#_#________###________________________
    #_#__#___##_#_#___#__#______###__#_#__#___###_##___#_______#___#_#___#_#_#__#____#__##__##__##___#_##
    ______##__##_###______##___#_##_#_#__##______#_#______##____#__#__#_#____#____##__#_#_###_____#__#_##
    _____#_____##__________##_#_##____#___##______#___#__#_#______###_#___________#________#_#__##__#____
    ##_##___#__#_#__#_###___##____##__#__#_#________#____#______##________#______#__#___#_#_#__#__#____#_
    _________#__________#___#_#__#_#__##____#______#____#_______#______##_____#__##___#___###__#_________
    ____#___##_##__#_____##__##____###___#__##_#_##_#____##__#__#__#_##___#_##_____#___________#__##__#__
    #_###______##__#_###___##_###__#__#___##__#_#___#__##__#__#__#_____#___#_____#__###_____#_________#__
    _#___#____#___#_#_____#_#__#_###_#___#_##_#____#__#__#_#_____#_#____##__________#_##_____#__##___#_#_
    _____#_______#__#___#_##____##_____#_________##___________#___#__##_#_#___###____#_##_#_#_#_###______
    __###_____#_#__#__#_______#____#__#__####___#_#_##__#_____#__#__#__#___#_____#_____##____##____#_____
    _#__#_#__#_#_______#_#_##_##______#_#_#___#__##_#_######__#___#_______#_#________#_________#_#_#_____
    #______#____#________________#__________#_____#________#___#___#_______##__#___##______#_______#__#__
    _#_###___#_##_________###__________#__#___#__#____###__#_#_#__#___#___#_____#__#__#_##_#_#_________#_
    _______#____#_____##___###_###____#_______#__#__#__#_______#__#_#_____#___#___________#__#__#___#____
    ______##______#_______##____#__##_#____#_##____#_##___#___##_#___#__##_________#____#___##_#_##____#_
    ______#______#__#_#__##_#___##_#_#________#__#_#_#_##_#__#____#___#_##________#__#_##_#_#____#_______
    _____##_________#_###___#_#___#__#_______####__#_______##__________________#_##____#__##_______#_###_
    ###__#_#____#__##_##_##_##_#_#_#_#_________###_______#_#___#_##___###____#_#__#_##_____#_____##_#__##
    ____####____#_______#__###___##__#___##____#_#____#___#_###_##_#____##____##_#_____#____#________#_#_
    #__#__#____#_#_#_#____#_____#______#__#_______####______#__##____#_____##______#_###__##__##_________
    _#_________#_____#_#_##__________#__#__#__#_______#___#___#____#__#_#_____#___###______#__#__#____#__
    #_#####____#____###_#_#_#_##___#_##___#___#_#______#______##__#_#___##____##_#_____#__#__##___##__#__
    ___#____#_____##__#___#___#________#___#_#_____#_##_#_##___#_#_#_#_#_#__________#_##_#__#_#_#_##_____
    #_#_###_____________##_#___##_____##___#____#_##_________________#____________#__________##_#________
    ____#_##_##______###_______#_#_#_#___####_______#___##___#_______##________##_##_#_#______#________##
    __#_#_____________#_#___#_____#_______#__#______#___#__###__#___A___#_#__#___#__#_###_##__#_#___###__
    _##_##______#_#____#__##_##___#_###__#______#___#_#_____#_#_______#__#__####_#__#____#_#______#__#___
    _#__#__##_____#__#__###____##____#_#__#_##____#_____#________#_#_#_#________#___#_##_____##_#______#_
    #______####__#_#____#_#______#____#__________#_#___#_#__#_____#_##_#___##_____#___##___#_____________
    _####____#__#________##_##____#_#_##__#__#__#_____#______#______#_#____#__#__##______#_________#_____
    #_#___#__##_#___#_____##__##___#_##__#_#__#__#_#__#____#______##___####_##_#___###_______##____#___#_
    #_#___#_#_#_#____#_#_#_##________#_______________##_#___#_____##___#_##__##_#___#____#___#__#____#___
    #____#_#_#_#_#_#_____##__________#_#________________###_____________#______###_###___##__##______#_##
    __#__#________#_______#_##__#____##_#_#__###__#____##__________#__________##_#____##_______##______#_
    _#__#_____#______#________#_#_#______#___##__________###____#_#_##___#__#___#_______##_______________
    #__#_#__#__#####___#_#___##_#__#_#__#___##__##_#_##_#_####__##__##____#_#__#_#_____#___##___________#
    #_#___#___#__#_#___#_#___#_##___#__#_____##___#_##_#_##____#__#_#____#__#____#_#__###__#_#______#___#
    #__#______#____#____#____##____#_______##_________#_____##____________#___####__##___#________#_#_#__
    #___#__#__###__##_#__##______#____##____##_____#________#____###____#_#__________#_________##______##
    ##_#_______#_##___#____#___######____#________####____#____###__#_#__#____##_#__#_____##______#_#____
    _##_###_#___##_#_#____#____#__#____#__#_#_____#__#____________#____##______###__##____#_______#_#___#
    #_______#_____#_#___#_##____##__#_#___##_#___##_#_________##__#_##__#__###___#______##_####___#_#____
    _____#_____#__#_____#_#________#__#____##_#___#______#___##__#____#__#__#___#____#_##___#####_____##_
    _##_______##_#______________#_#__#_###__#__##__#________##__#_____________#_______#_#________#_#___#_
    ______#____###__#_____#____##_##_#__#_____#____#____#_#___#____#__#__________#_#__##___#________##__#
    ______________##____#__#_#__##_____________________#___##_____#____#_#_______#_#_#______#____#____#__
    _#_#_____#____#__#_____##_##_____#___##___###_##____#_#__#____####_#________#_#_#___##___#_#____#____
    _#_____#####_#____####__#______#_##____#__##_______#_#__#_##__##______#_#__#___##______#_##_##___#___
    ______#_#___#_#_#_#_#__#____#____#_#_#_________#_#___##_____#______#____###__#_________##__##_____##_
    ______#__##______#_#_#__#______##_#_#_##__##__##__#_______#_____________________##__#_#__#__#____#__#
    _____##___#_#______#____#__##_#____#_#_____#_##_#_#____#_#____###__#________#_###__#_#__###_#__#_##__
    #_____##_#__###___#_________#__#______#____#__#____##______#___###____#__#____####_#___##_##___#_____
    ____#_#_#___#_#_#___#_##_##_#___####_#_#_______#_#__________#____#_#_#___#___________#_____###______#
    _#___##_#__#__#_#_______#_#___#____##_#__#____##___####____________##_#_#____#__#___#__#___#__#____#_
    #_#___#_##_#____##___###____###______#___##____#_#__#____#_______##_##___#_____#_##_____##__#____##__
    ______#_#_#_#____#_#____#_##____#_________#_#__##__#_#####____##_______###__#_#____##____#_#__##____#
    _____#__#___###_#________T#___#___________##_#__________#__##___###__#_#__###_#___#__#_#___#__#__#___
    __________________#___#__#_____#_##__#____#______##_______#____#_#___#__________#__##________###___#_
    ______##__#____#_#____##___#____##_#______#_#_##_#____##_#__###_#___#__#____#___#__#___#_#__##_______
    #_##_#____________#__#_______#_____#_#_##__#__#_###___##__#_##_#__##___###__#____##______#___##_##___
    _______#______#_#_#______#_##_##________#__#___##__________#___#_#_###_#__#__#_#____#___#__##__#_____
    _##_#________#_______#_#___##___#_____#__##_#_#______#___##__#_#_____________#__#_____#__##___#__#___
    ##__#_____###__#_##___________##__#__#_____#_______________#__##___#_____###_#___##_#_##__#____###_#_
    _____#___##____##_____###____#_#__#___#__________##__##___#___#_##_#__#_______#_#________#___#_______
    _#___##___#_____________#_#__#______#______#_#_______________##___#____#___#_#______#_##___#____#_#_#
    __####_#_#___######_#_#_____#_#_#_#___#_#__###__#_##______#____###_#______#_#___#__#__##_#__##___#_#_
    __##__##_____#____#___###______#_#_##_#__##____#________#_____#___#____###______#_#______#___##_#_#_#
    _#_________#________#_____#___#__#_#__##__#_####___#_##_#_#_##_#_#___#______#____##____###_#_____#___
    _##___#___#_#_#_#_##___#_#__________###_#_##_________#_#__#__#_#______##___##_____#______#__#______##
    _#_______#_##_##_____#___##____##___#_##___#__#_######_________#__#__#_#_#_________________#_#_____#_
    #_________#__#_#_##___#__###_____#______#_________##_______#___#____##_#____#________#___#__##_#_#___
    #_#____#____#___##____##_#_#_#####______##_#_#____#_#____#_#__#___#___##__###___##___#__##___________
    ##______#_____#_#_#___#_____###__#_________#_##_#_____##________#_#_##___#__#___#___#______#____#_#__
    ______#_#___#_____#_____#_#___#____#___###_#___#_#_____#__#___###________#__###____#__#_#_____##_____
    _#_##___#__##_#_#____#_#__#______#_#_________#__#___##__#__#___#__##__#____#_______#____#_##_#__#____
    ______________##______#_______####_#__####_#_####___##_##_#_#______#___##__#__#___________#_#____###_
    ###_____#______#_#_____#__###__#_####_#______#__##_____#_#___________##__________#_____##____________
    ____#___##_#____#__#_______#____##______#____#_####_____#___#____#__________#_#____#__#______#__##___
    _#___#___#__#____#_#_#___#__##_______________##___#__#____#_#_________#__#____##_#______##_______#___
    _#____###___#_#___##_##___####______##___#____#_##___#___#_#__###_______#_______#__#_#___#__###_#____
    #_#___#_#___#____##_#_#____#___#__###_##___##___#_#_#_##_#_#__#___#__##_#___#__#____#___#_#__#_#_#__#
    #_##____#___#____#___#____#_#________#__#__#_#______#_____#____#_#______#____#___#____#_#____##______
    """
    # real_grid = [
    #     ['_', '_', '_', '_', '#'],
    #     ['_', '#', '_', '#', '_'],
    #     ['_', '#', 'A', '_', '_'],
    #     ['_', '#', '_', '#', '_'],
    #     ['#', '_', '_', '_', 'T']
    # ]

    cleaned_grid = asci_grid.strip().split("\n")

    grid_example = []

    for row in cleaned_grid:
        grid_example.append([char if char in {'#', '_', 'A', 'T'} else '_' for char in row])
    
    
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
        visualize_maze_debug(grid_example, path, None, title="Repeated Forward A* Debug Visualization")
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

def test_adaptive_a_star():
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

    # Run Adaptive A* Algorithm
    path, expanded_nodes, heuristic = adaptive_a_star(grid_example, start, goal)
    print(f"Path found: {path}")
    print(f"Total expanded nodes: {(expanded_nodes)}")
    print(f"Updated Heuristic: {heuristic}")

#test_adaptive_a_star()
#test_repeated_backward_astar()
#test_repeated_forward_astar()
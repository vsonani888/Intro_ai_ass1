import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import os
import random

from binaryheapbyus import BinaryHeap  # Your custom BinaryHeap class

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

def reconstruct_path(came_from, start, goal):
    """Reconstruct path given a dictionary of 'came_from' pointers."""
    path = []
    current = goal
    while current in came_from or current == start:
        path.append(current)
        if current == start:
            break
        current = came_from[current]
    path.reverse()
    if path and path[0] == start:
        return path
    return None  # No valid path found

def forward_a_star(known_grid, start, goal, tie_break='LARGER_G'):
    """
    A single run of A* on the agent's current known grid.
    Returns (path, expanded_set).
      - path: list of cells from 'start' to 'goal' if found, else None
      - expanded_set: set of cells expanded (for stats)
    """
    grid_size = len(known_grid)
    open_list = BinaryHeap()
    
    g_score = {start: 0}
    f_score = {start: manhattan_distance(start, goal)}
    came_from = {}
    
    # Push to open list, with tie-breaking on g if requested
    if tie_break == 'LARGER_G':
        open_list.push((f_score[start], -g_score[start], random.random(), start))
    else:
        open_list.push((f_score[start], g_score[start], random.random(), start))
    
    expanded = set()  # For stats, store which nodes get expanded
    
    while not open_list.is_empty():
        _, _, _, current = open_list.pop()
        if current in expanded:
            continue
        expanded.add(current)
        
        # Goal check
        if current == goal:
            path = reconstruct_path(came_from, start, goal)
            return path, expanded
        
        # Expand neighbors
        for neighbor in get_neighbors(current, grid_size):
            r, c = neighbor
            # Skip if known blocked
            if known_grid[r][c] == '#':
                continue
            
            tentative_g = g_score[current] + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + manhattan_distance(neighbor, goal)
                
                if tie_break == 'LARGER_G':
                    open_list.push((f_score[neighbor], -g_score[neighbor], random.random(), neighbor))
                else:
                    open_list.push((f_score[neighbor], g_score[neighbor], random.random(), neighbor))
    
    # No path found
    return None, expanded

def reveal_surroundings(real_grid, known_grid, position):
    """
    Simulate 'discovering' the true status of cells around the agent.
    For simplicity, we reveal the 4 neighbors and the current cell.
    """
    for cell in [position] + get_neighbors(position, len(real_grid)):
        r, c = cell
        known_grid[r][c] = real_grid[r][c]

def plot_grid_with_path(known_grid, path, start, goal, iteration, output_folder, title_prefix="Iteration"):
    """
    Create a color-coded plot of the known grid and save it to a file.
    
    Color mapping for cells:
      '#' -> 0 (black)
      '$' -> 1 (dark gray)
      '_' -> 2 (white)
      'p' -> 3 (blue)
      'A' -> 4 (green)
      'T' -> 5 (red)
    """
    n = len(known_grid)
    
    # Make a copy so we don't mutate the original
    grid_copy = [row[:] for row in known_grid]
    
    # Mark the path
    if path:
        for (r, c) in path:
            # Skip marking the start or goal as 'p'
            if (r, c) not in (start, goal):
                grid_copy[r][c] = 'p'
    
    # Mark start and goal
    sr, sc = start
    gr, gc = goal
    grid_copy[sr][sc] = 'A'
    grid_copy[gr][gc] = 'T'
    
    # Convert cell symbols to integer codes
    color_map = {
        '#': 0,  # wall -> black
        '$': 1,  # unknown -> dark gray
        '_': 2,  # free -> white
        'p': 3,  # path -> blue
        'A': 4,  # agent -> green
        'T': 5   # goal -> red
    }
    data = np.zeros((n, n), dtype=int)
    for i in range(n):
        for j in range(n):
            data[i, j] = color_map.get(grid_copy[i][j], 2)  # default to 2 (white)
    
    # Define custom colormap
    cmap = ListedColormap([
        "black",      # 0 = wall
        "dimgray",    # 1 = unknown
        "white",      # 2 = free cell
        "blue",       # 3 = path
        "green",      # 4 = agent
        "red"         # 5 = goal
    ])
    
    fig, ax = plt.subplots(figsize=(5, 5))
    ax.imshow(data, cmap=cmap, vmin=0, vmax=5)
    
    ax.set_title(f"{title_prefix} {iteration}")
    ax.set_xticks([])
    ax.set_yticks([])
    
    # Ensure output folder exists
    os.makedirs(output_folder, exist_ok=True)
    
    # Save figure to the folder
    plt.savefig(os.path.join(output_folder, f"iteration_{iteration}.png"), bbox_inches='tight')
    plt.close(fig)

def repeated_forward_a_star(real_grid, start, goal, tie_break='LARGER_G', output_folder="astar_iterations"):
    """
    Repeated Forward A* with matplotlib-based visualization.
    * We keep a 'known_grid' which starts unknown ('$') everywhere.
    * The agent repeatedly plans a path and moves until the goal is reached or no path is possible.
    * We save a snapshot each time a plan is generated.
    * Once the goal is reached, we save one EXTRA snapshot showing the final route
      (the actual step-by-step path the agent physically took).
    """
    n = len(real_grid)
    
    # Initialize known grid as unknown
    known_grid = [['$' for _ in range(n)] for _ in range(n)]
    
    # We'll store every step that the agent *actually takes* in agent_trajectory.
    agent_trajectory = [start]  # The agent starts at 'start'
    
    agent_pos = start
    reveal_surroundings(real_grid, known_grid, agent_pos)
    
    total_expanded = 0
    iteration_count = 0
    
    while True:
        # 1) Plan a path in the known grid from agent_pos to goal
        path, expanded_nodes = forward_a_star(known_grid, agent_pos, goal, tie_break)
        total_expanded += len(expanded_nodes)
        
        iteration_count += 1
        # Show the newly planned path (but only from agent's vantage, not the entire route)
        plot_grid_with_path(
            known_grid, path, agent_pos, goal, 
            iteration=iteration_count, 
            output_folder=output_folder,
            title_prefix="Planning"
        )
        
        if not path:
            print(f"No path found at iteration {iteration_count}. Terminating.")
            # Extra snapshot not needed because we never reached the goal
            return None, total_expanded
        
        # 2) Move along the path until we either reach the goal or discover a blocked cell
        stuck = False
        for i in range(1, len(path)):
            next_cell = path[i]
            
            # "Move" agent
            agent_pos = next_cell
            # Record the actual step
            agent_trajectory.append(agent_pos)
            
            # Reveal surroundings from this new position
            reveal_surroundings(real_grid, known_grid, agent_pos)
            
            # If the newly revealed cell is blocked, we are stuck -> replan next
            if known_grid[agent_pos[0]][agent_pos[1]] == '#':
                stuck = True
                break
            
            # Check if we've reached the goal
            if agent_pos == goal:
                print(f"Goal reached at iteration {iteration_count}!")
                
                # FINAL SNAPSHOT: show the actual route traveled from the original start
                iteration_count += 1
                plot_grid_with_path(
                    known_grid, agent_trajectory, start, goal,
                    iteration=iteration_count,
                    output_folder=output_folder,
                    title_prefix="Final Route"
                )
                return agent_trajectory, total_expanded
        
        # If we got here without returning, the path ended but we didn't necessarily reach the goal
        if stuck:
            print(f"Agent got stuck at {agent_pos}; discovered a wall. Replanning...\n")
            # We'll loop back to re-plan from current position
            continue
        
        # If we walked the full path and ended exactly on the goal,
        # we would have returned inside the loop. So if we are here,
        # we haven't reached the goal but used up the path.
        if agent_pos == goal:
            # Safety check—should rarely happen outside the loop above
            print(f"Goal unexpectedly reached at iteration {iteration_count}.")
            iteration_count += 1
            plot_grid_with_path(
                known_grid, agent_trajectory, start, goal,
                iteration=iteration_count,
                output_folder=output_folder,
                title_prefix="Final Route"
            )
            return agent_trajectory, total_expanded
        
        # Otherwise, we continue and plan again from agent_pos


def test_repeated_forward_astar():
    """
    Example usage of repeated_forward_a_star with a small grid:
      - 'A' marks the start
      - 'T' marks the goal
      - '#' marks blocked cells
      - '_' marks free cells
    """

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

    rows = len(cleaned_grid)
    cols = len(cleaned_grid[0])

    real_grid = []

    for row in cleaned_grid:
        real_grid.append([char if char in {'#', '_', 'A', 'T'} else '_' for char in row])
    
    # Identify start and goal
    start, goal = None, None
    for i in range(len(real_grid)):
        for j in range(len(real_grid[0])):
            if real_grid[i][j] == 'A':
                start = (i, j)
                real_grid[i][j] = '_'  # so it is consistent with free cell
            elif real_grid[i][j] == 'T':
                goal = (i, j)
                real_grid[i][j] = '_'  # so it is consistent with free cell
    
    path, total_expanded = repeated_forward_a_star(real_grid, start, goal, tie_break='LARGER_G', output_folder="astar_iterations")
    print("==== Final Result ====")
    if path:
        print(f"Found a path of length {len(path)} with total expanded = {total_expanded}")
        print("Final path (agent's trajectory) =", path)
    else:
        print("No path could be found.")

if __name__ == "__main__":
    test_repeated_forward_astar()

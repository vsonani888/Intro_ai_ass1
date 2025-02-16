import heapq
import numpy as np
import matplotlib.pyplot as plt
import time

class PriorityQueue:
    """Min-heap priority queue to handle open list."""
    def __init__(self):
        self.elements = []

    def push(self, priority, count, item):
        """Push an item with a given priority into the heap."""
        heapq.heappush(self.elements, (priority, -count, item))  # -count ensures larger g-values are prioritized

    def pop(self):
        """Pop the item with the highest priority (smallest f-value, largest g-value)."""
        return heapq.heappop(self.elements)[2]

    def is_empty(self):
        return len(self.elements) == 0

def manhattan_distance(a, b):
    """Computes the Manhattan distance heuristic between two points."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def get_neighbors(position, grid_size):
    """Returns valid neighbors in the 4-directional grid."""
    r, c = position
    neighbors = []
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # Up, Down, Left, Right
        nr, nc = r + dr, c + dc
        if 0 <= nr < grid_size and 0 <= nc < grid_size:
            neighbors.append((nr, nc))
    return neighbors

def reconstruct_path(came_from, start, goal):
    """Reconstructs the path from goal to start using backtracking."""
    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path

def repeatedForwardAStarB(grid, start, goal, visualize=False):
    """
    Implements Repeated Forward A* with tie-breaking favoring larger g-values.
    
    Arguments:
    grid -- 2D numpy array representing the grid world (1=unblocked, 2=blocked)
    start -- (row, col) starting position
    goal -- (row, col) target position
    visualize -- boolean to enable visualization
    
    Returns:
    path -- List of tuples representing the path taken
    expanded_nodes -- Total number of expanded nodes
    """
    grid_size = grid.shape[0]
    g_values = {start: 0}  # g(s): cost from start to each node
    came_from = {}  # Stores path reconstruction
    total_expanded = 0  # Tracks number of expanded nodes

    # Priority queue (open list), initialized with (f-value, -g-value, node)
    open_list = PriorityQueue()
    f_start = manhattan_distance(start, goal)  # f(s) = g(s) + h(s)
    open_list.push(f_start, g_values[start], start)

    while not open_list.is_empty():
        current = open_list.pop()
        total_expanded += 1

        if current == goal:  # Goal reached
            return reconstruct_path(came_from, start, goal), total_expanded

        for neighbor in get_neighbors(current, grid_size):
            if grid[neighbor] == 2:  # Skip blocked cells
                continue
            
            tentative_g = g_values[current] + 1  # Move cost is always 1
            if neighbor not in g_values or tentative_g < g_values[neighbor]:
                g_values[neighbor] = tentative_g
                f_value = tentative_g + manhattan_distance(neighbor, goal)
                open_list.push(f_value, g_values[neighbor], neighbor)  # Prioritize larger g-values
                came_from[neighbor] = current

    return None, total_expanded  # No path found

# Example Usage
if __name__ == "__main__":
    grid_size = 10
    grid = np.ones((grid_size, grid_size))  # 1 = unblocked, 2 = blocked

    # Sample obstacles
    obstacles = [(3, 3), (3, 4), (4, 4), (5, 4)]
    for obs in obstacles:
        grid[obs] = 2  # Mark blocked cells

    start = (0, 0)
    goal = (9, 9)

    path, expanded_nodes = repeatedForwardAStarB(grid, start, goal, visualize=True)

    if path:
        print("Path Found:", path)
        print("Nodes Expanded:", expanded_nodes)
    else:
        print("No Path Found")

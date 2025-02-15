import random

def get_unvisited_neighbors(row, col, grid, visited):
    """
    Returns the list of unvisited neighbors (up, down, left, right)
    for the cell (row, col).
    """
    neighbors = []
    max_row = len(grid)
    max_col = len(grid[0])
    
    # Up
    if row - 1 >= 0 and not visited[row - 1][col]:
        neighbors.append((row - 1, col))
    # Down
    if row + 1 < max_row and not visited[row + 1][col]:
        neighbors.append((row + 1, col))
    # Left
    if col - 1 >= 0 and not visited[row][col - 1]:
        neighbors.append((row, col - 1))
    # Right
    if col + 1 < max_col and not visited[row][col + 1]:
        neighbors.append((row, col + 1))
    
    return neighbors

def generate_maze_10x10():
    """
    Generates a 10x10 maze/corridor-like grid using a DFS-based approach.
    Each new cell has a 30% chance of being blocked (#) and 70% chance unblocked (_).
    Then places 'A' (agent) and 'T' (target) on random unblocked cells.
    """
    size = 20
    
    # Initialize the grid with placeholders
    grid = [['?' for _ in range(size)] for _ in range(size)]
    visited = [[False for _ in range(size)] for _ in range(size)]
    
    # We will use a stack for DFS
    stack = []
    
    def all_visited(visited_grid):
        """Checks if all cells have been visited."""
        for r in range(size):
            for c in range(size):
                if not visited_grid[r][c]:
                    return False
        return True
    
    while not all_visited(visited):
        # If stack is empty, pick a random unvisited cell as a new start
        if not stack:
            unvisited_cells = [(r, c) for r in range(size) for c in range(size) if not visited[r][c]]
            start_cell = random.choice(unvisited_cells)
            stack.append(start_cell)
            
            # Mark the start cell as visited and unblocked
            sr, sc = start_cell
            visited[sr][sc] = True
            grid[sr][sc] = '_'
        
        # Peek at the top cell in the stack
        current = stack[-1]
        r, c = current
        
        # Get unvisited neighbors
        neighbors = get_unvisited_neighbors(r, c, grid, visited)
        
        if not neighbors:
            # No unvisited neighbors -> dead end, backtrack
            stack.pop()
        else:
            # Choose a random unvisited neighbor
            nr, nc = random.choice(neighbors)
            visited[nr][nc] = True
            
            # Decide if blocked or unblocked
            if random.random() < 0.3:
                # 30% chance blocked
                grid[nr][nc] = '#'
                # Don't push blocked cell onto the stack
            else:
                # 70% chance unblocked
                grid[nr][nc] = '_'
                stack.append((nr, nc))

    # Now place A (agent) and T (target) in random unblocked cells
    unblocked_positions = [(r,c) for r in range(size) for c in range(size) if grid[r][c] == '_']
    
    if len(unblocked_positions) < 2:
        # In case there are not enough unblocked cells (edge case)
        print("Not enough unblocked cells to place agent and target!")
        return
    
    # Randomly choose a cell for the agent 'A'
    agent_cell = random.choice(unblocked_positions)
    grid[agent_cell[0]][agent_cell[1]] = 'A'
    unblocked_positions.remove(agent_cell)
    
    # Randomly choose a different cell for the target 'T'
    target_cell = random.choice(unblocked_positions)
    grid[target_cell[0]][target_cell[1]] = 'T'
    
    # Print the maze
    for row in grid:
        print(''.join(row))

if __name__ == "__main__":
    generate_maze_10x10()

import random

def generate_maze_101x101():
    size = 101
    grid = [['?' for _ in range(size)] for _ in range(size)]
    visited = [[False for _ in range(size)] for _ in range(size)]
    stack = []

    def all_visited():
        for row in visited:
            if False in row:
                return False
        return True

    def get_unvisited_neighbors(r, c):
        neighbors = []
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < size and 0 <= nc < size and not visited[nr][nc]:
                neighbors.append((nr, nc))
        return neighbors

    while not all_visited():
        if not stack:
            unvisited_cells = [(r, c) for r in range(size) for c in range(size) if not visited[r][c]]
            start_cell = random.choice(unvisited_cells)
            stack.append(start_cell)
            visited[start_cell[0]][start_cell[1]] = True
            grid[start_cell[0]][start_cell[1]] = '_'
        
        current = stack[-1]
        neighbors = get_unvisited_neighbors(*current)
        
        if not neighbors:
            stack.pop()
        else:
            neighbor = random.choice(neighbors)
            visited[neighbor[0]][neighbor[1]] = True
            if random.random() < 0.3:
                grid[neighbor[0]][neighbor[1]] = '#'  # Blocked
            else:
                grid[neighbor[0]][neighbor[1]] = '_'  # Unblocked
                stack.append(neighbor)

    unblocked_cells = [(r, c) for r in range(size) for c in range(size) if grid[r][c] == '_']
    if len(unblocked_cells) < 2:
        return generate_maze_101x101()  # Regenerate if not enough unblocked cells

    agent_cell = random.choice(unblocked_cells)
    grid[agent_cell[0]][agent_cell[1]] = 'A'
    unblocked_cells.remove(agent_cell)

    target_cell = random.choice(unblocked_cells)
    grid[target_cell[0]][target_cell[1]] = 'T'

    return grid

def save_mazes(num_mazes=50, filename='generated_mazes_101.txt'):
    with open(filename, 'w') as file:
        for i in range(num_mazes):
            maze = generate_maze_101x101()
            file.write(f"MAZE {i}\n")
            for row in maze:
                file.write(''.join(row) + '\n')
            file.write('\n')  # Blank line between mazes
    print(f"Successfully generated and saved {num_mazes} mazes.")

# Generate and save 50 mazes
save_mazes()

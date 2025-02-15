import random

def generate_gridworld(rows=101, cols=101, block_prob=0.30):
    """
    Generates a single gridworld of size rows×cols using a DFS-based
    approach. Each new cell is blocked with probability block_prob
    and unblocked otherwise.
    
    Args:
        rows (int): Number of rows in the grid.
        cols (int): Number of columns in the grid.
        block_prob (float): Probability that a newly discovered cell is blocked.
    
    Returns:
        grid (list[list[bool]]): A 2D list where True=blocked, False=unblocked.
    """
    grid = [[False for _ in range(cols)] for _ in range(rows)]    # Default unblocked
    visited = [[False for _ in range(cols)] for _ in range(rows)]

    def get_unvisited_neighbors(r, c):
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # up, down, left, right
        return [
            (nr, nc) for (dr, dc) in directions
            for nr, nc in [(r + dr, c + dc)]
            if 0 <= nr < rows and 0 <= nc < cols and not visited[nr][nc]
        ]

    while True:
        # Find any unvisited cell
        unvisited_cells = [(r, c) for r in range(rows) for c in range(cols) if not visited[r][c]]
        if not unvisited_cells:
            # All cells visited
            break

        # Pick a random unvisited cell to start DFS
        start_r, start_c = random.choice(unvisited_cells)
        visited[start_r][start_c] = True
        grid[start_r][start_c] = False  # force unblocked for DFS entrance

        # DFS stack
        stack = [(start_r, start_c)]
        while stack:
            r, c = stack[-1]
            neighbors = get_unvisited_neighbors(r, c)

            if neighbors:
                nr, nc = random.choice(neighbors)
                visited[nr][nc] = True
                if random.random() < block_prob:
                    # Blocked
                    grid[nr][nc] = True
                else:
                    # Unblocked => continue DFS
                    grid[nr][nc] = False
                    stack.append((nr, nc))
            else:
                stack.pop()

    return grid

def generate_and_save_mazes_txt(
    num_mazes=50, 
    rows=101, 
    cols=101, 
    block_prob=0.30, 
    output_file="all_mazes.txt"
):
    """
    Generates 'num_mazes' maze-like grids and writes them all into a single
    text file, separated by blank lines. Each maze is labeled 'MAZE i'.
    
    Args:
        num_mazes (int): How many mazes to generate.
        rows (int): Grid rows.
        cols (int): Grid columns.
        block_prob (float): Probability a visited cell is blocked.
        output_file (str): Path to the text file for saving all mazes.
    """
    with open(output_file, 'w') as f:
        for i in range(num_mazes):
            grid = generate_gridworld(rows, cols, block_prob)
            f.write(f"MAZE {i}\n")
            for row in grid:
                line_str = "".join('1' if cell else '0' for cell in row)
                f.write(line_str + "\n")
            f.write("\n")  # blank line separator
            print(f"Maze {i} generated.")

    print(f"\nAll {num_mazes} mazes saved to '{output_file}'.")

if __name__ == "__main__":
    # Generate 50 mazes and store them all into 'all_mazes.txt'
    generate_and_save_mazes_txt(
        num_mazes=5,
        rows=5,
        cols=5,
        block_prob=0.30,
        output_file="all_mazes_101.txt"
    )

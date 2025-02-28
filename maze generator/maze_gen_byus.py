import random

def generate_maze(size):
    #print(size)

    grid = [['?' for _ in range(size)] for _ in range(size)] # initialize a list with ? to mark it as unvisited
    visited = [[False for _ in range(size)] for _ in range(size)] # make a list for cells that are unvisited

    stack = [] # stack for DFS


    while not all_visited(visited): #loop ends if it cannot find any unvisited cells in the visited list
        if not stack :

            unvisited_cells = unvisited_maker(visited)
            start_cell = random.choice(unvisited_cells)
            stack.append(start_cell)

            start_row = start_cell[0]
            start_column = start_cell[1]

            visited[start_row][start_column] = True
            grid[start_row][start_column] = '_'

        current = stack[len(stack)-1]
        current_row = current[0]
        current_column = current[1]

        neighbours = get_unvisited_neighbours(current_row, current_column, grid, visited)

        if not neighbours: # dead end
            stack.pop()

        else: # has neighbours
            random_neighbour = random.choice(neighbours)
            neighbours_row = random_neighbour[0]
            neighbours_column = random_neighbour[1]

            visited[neighbours_row][neighbours_column] = True

            if random.random() < 0.3:
                grid[neighbours_row][neighbours_column] = '#'
            else:
                grid[neighbours_row][neighbours_column] = '_'
                stack.append((neighbours_row, neighbours_column))


    unblocked_cells = get_unblocked_cells(grid)
    
    agent_cell = random.choice(unblocked_cells)
    grid[agent_cell[0]][agent_cell[1]] = 'A'
    unblocked_cells.remove(agent_cell)

    target_cell = random.choice(unblocked_cells)
    grid[target_cell[0]][target_cell[1]] = 'T'

    print_maze(grid)

def get_unblocked_cells(grid):
    unblocked_cells = []
    size = len(grid)

    for row in range(size):
        for column in range(size):
            if grid[row][column] == '_':
                unblocked_cells.append((row, column))
    
    return unblocked_cells

def get_unvisited_neighbours(r, c, grid, visited): #get list of unvisited neighbours
    
    neighbors = []
    size = len(grid)

    if r - 1 >= 0 and not visited[r - 1][c]:
        neighbors.append((r - 1, c))

    if r + 1 < size and not visited[r + 1][c]:
        neighbors.append((r + 1, c))

    if c - 1 >= 0 and not visited[r][c - 1]:
        neighbors.append((r, c - 1))

    if c + 1 < size and not visited[r][c + 1]:
        neighbors.append((r, c + 1))

    return neighbors

    

def unvisited_maker(visited): #loops over the visited list and finds and returns list of unvisited cells

    unvisited_cells = []
    size = len(visited)

    for row in range(size):
        for column in range(size):
            if not visited[row][column]:
                unvisited_cells.append((row, column))
    
    return unvisited_cells

def all_visited (visited): # #loops over the visited list and finds and returns if there are any unvisited cells
    for row in range(len(visited)):
        for column in range(len(visited)):
            if not visited[row][column]:
                return False
    return True

def print_maze(grid): #loops over the grid list and prints it out
    for row in range(len(grid)):
        for column in range(len(grid)):
            print(grid[row][column], end="", file = file)
        print(file = file)

if __name__ == "__main__":
    
    print("hi from main")

    size = 10
    maze_qty = 20
    file = open("generated_mazes_10.txt", "w")

    #print("printing ", maze_qty, " mazes", file = file)

    #generate_maze(size)

    for i in range(maze_qty):
        print("MAZE ", i, file = file)
        generate_maze(size)
        print("", file = file)

    file.close
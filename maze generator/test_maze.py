import matplotlib.pyplot as plt
import numpy as np

# Define the maze from the given string representation
maze_string = """
#__#_
#_#_T
##___
___A#
###_#
"""

# Convert the string representation into a 2D list
maze_lines = maze_string.strip().split("\n")
maze = np.array([list(line) for line in maze_lines])

# Define size of the maze
rows, cols = maze.shape

# Create a plot
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xticks([])
ax.set_yticks([])

# Define colors for different elements in the maze
color_map = {
    "#": "black",  # Walls
    "_": "white",  # Paths
    "A": "red",    # End point (Exit)
    "T": "blue"    # Start point (Entry)
}

# Draw the maze
for r in range(rows):
    for c in range(cols):
        color = color_map.get(maze[r, c], "white")
        ax.add_patch(plt.Rectangle((c, rows - r - 1), 1, 1, color=color))

# Display the maze
ax.set_xlim(0, cols)
ax.set_ylim(0, rows)
plt.gca().set_aspect('equal')
plt.show()

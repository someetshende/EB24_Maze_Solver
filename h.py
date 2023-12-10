import heapq
import tkinter as tk
from tkinter import ttk

# Constants
WIDTH, HEIGHT = 800, 600
GRID_SIZE = 20
WHITE = "white"
BLACK = "black"
GREEN = "green"
RED = "red"
BLUE = "blue"

# Node class to represent each cell in the grid
class Node:
    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.neighbors = []
        self.is_obstacle = False
        self.is_start = False
        self.is_end = False
        self.g_cost = float("inf")
        self.h_cost = float("inf")
        self.f_cost = float("inf")
        self.previous = None

    def __lt__(self, other):
        return self.f_cost < other.f_cost


# Function to initialize the grid
def initialize_grid(rows, cols):
    grid = [[Node(row, col) for col in range(cols)] for row in range(rows)]
    return grid


# Function to draw the nodes
def draw_nodes(canvas, grid):
    for row in grid:
        for node in row:
            color = WHITE
            if node.is_obstacle:
                color = BLACK
            elif node.is_start:
                color = GREEN
            elif node.is_end:
                color = RED
            canvas.create_rectangle(
                node.col * GRID_SIZE,
                node.row * GRID_SIZE,
                (node.col + 1) * GRID_SIZE,
                (node.row + 1) * GRID_SIZE,
                fill=color,
            )


# Dijkstra's Algorithm
def dijkstra(grid, start, end):
    open_set = [start]
    start.g_cost = 0

    while open_set:
        current_node = heapq.heappop(open_set)

        if current_node == end:
            return reconstruct_path(end)

        for neighbor in current_node.neighbors:
            temp_g_cost = current_node.g_cost + 1

            if temp_g_cost < neighbor.g_cost:
                neighbor.previous = current_node
                neighbor.g_cost = temp_g_cost
                neighbor.f_cost = neighbor.g_cost
                if neighbor not in open_set:
                    heapq.heappush(open_set, neighbor)

        canvas.update()

    return None


# A* Algorithm
def astar(grid, start, end):
    open_set = [start]
    start.g_cost = 0
    start.h_cost = calculate_h_cost(start, end)
    start.f_cost = start.g_cost + start.h_cost

    while open_set:
        current_node = heapq.heappop(open_set)

        if current_node == end:
            return reconstruct_path(end)

        for neighbor in current_node.neighbors:
            temp_g_cost = current_node.g_cost + 1

            if temp_g_cost < neighbor.g_cost:
                neighbor.previous = current_node
                neighbor.g_cost = temp_g_cost
                neighbor.h_cost = calculate_h_cost(neighbor, end)
                neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
                if neighbor not in open_set:
                    heapq.heappush(open_set, neighbor)

        canvas.update()

    return None


# Function to calculate the heuristic cost (Euclidean distance)
def calculate_h_cost(node, end):
    return ((node.row - end.row) ** 2 + (node.col - end.col) ** 2) ** 0.5


# Function to reconstruct the path from end to start
def reconstruct_path(node):
    path = []
    current = node
    while current:
        path.append(current)
        current = current.previous
    return path[::-1]


# Function to handle mouse clicks
def handle_click(event):
    row = event.y // GRID_SIZE
    col = event.x // GRID_SIZE
    node = grid[row][col]

    if not node.is_start and not node.is_end:
        node.is_obstacle = not node.is_obstacle
        draw_nodes(canvas, grid)


# Function to run the selected algorithm
def run_algorithm():
    start_node = None
    end_node = None

    for row in grid:
        for node in row:
            if node.is_start:
                start_node = node
            elif node.is_end:
                end_node = node

    if start_node and end_node:
        if algorithm_var.get() == "Dijkstra":
            path = dijkstra(grid, start_node, end_node)
        elif algorithm_var.get() == "A*":
            path = astar(grid, start_node, end_node)

        if path:
            for node in path:
                if not node.is_start and not node.is_end:
                    node.is_obstacle = False
            draw_nodes(canvas, grid)


# Main function
def main():
    global canvas, grid, algorithm_var

    root = tk.Tk()
    root.title("Pathfinding Algorithms")

    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT)
    canvas.pack()

    rows = HEIGHT // GRID_SIZE
    cols = WIDTH // GRID_SIZE

    grid = initialize_grid(rows, cols)

    start_node = grid[10][5]
    end_node = grid[rows - 10][cols - 5]

    start_node.is_start = True
    end_node.is_end = True

    # Connect nodes to their neighbors
    for row in grid:
        for node in row:
            if node.row > 0:
                node.neighbors.append(grid[node.row - 1][node.col])
            if node.row < rows - 1:
                node.neighbors.append(grid[node.row + 1][node.col])
            if node.col > 0:
                node.neighbors.append(grid[node.row][node.col - 1])
            if node.col < cols - 1:
                node.neighbors.append(grid[node.row][node.col + 1])

    draw_nodes(canvas, grid)
    canvas.bind("<Button-1>", handle_click)

    algorithm_var = tk.StringVar()
    algorithm_var.set("Dijkstra")

    algorithm_menu = ttk.Combobox(root, textvariable=algorithm_var, values=["Dijkstra", "A*"])
    algorithm_menu.pack()

    run_button = tk.Button(root, text="Run Algorithm", command=run_algorithm)
    run_button.pack()

    root.mainloop()


if __name__ == "__main__":
    main()

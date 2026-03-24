import numpy as np
import matplotlib.pyplot as plt
import random

# creating map
# random nxn grid w/ obstacles
class GridWorld:
    def __init__(self, size, obstacle_prob=0.25):
        self.size = size
        # the actual world
        self.true_map = np.random.rand(size, size) < obstacle_prob
        self.true_map[0, 0] = False
        self.true_map[size-1, size-1] = False

    def is_obstacle(self, x, y):
        return self.true_map[x, y]

# DFS algorithm
class DFSPlanner:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.visited = set()
        self.parent = {}

    def neighbors(self, node):
        x, y = node
        moves = [(1,0),(-1,0),(0,1),(0,-1)]
        random.shuffle(moves)  # make it more reflective of a rover

        result = []
        for dx, dy in moves:
            nx, ny = x+dx, y+dy
            if 0 <= nx < self.grid.size and 0 <= ny < self.grid.size:
                if not self.grid.is_obstacle(nx, ny):
                    result.append((nx, ny))
        return result

    def search(self):
        stack = [self.start]
        self.visited.add(self.start)

        while stack:
            current = stack.pop()

            if current == self.goal:
                return True

            for n in self.neighbors(current):
                if n not in self.visited:
                    self.visited.add(n)
                    self.parent[n] = current
                    stack.append(n)

        return False

    def get_path(self):
        path = []
        node = self.goal
        while node != self.start:
            path.append(node)
            node = self.parent[node]
        path.append(self.start)
        return path[::-1]

# main
size = 30
world = GridWorld(size)
start = (0,0) # you can change but why
goal = (size-1,size-1)

planner = DFSPlanner(world, start, goal)

# check just in case path is impossible
found = planner.search()
if not found:
    print("No path found!")
    exit()

path = planner.get_path()

# PLOTTING PATH
print(f"DFS path length: {len(path)}")
img = np.zeros((size, size, 3))
img[world.true_map] = [0,0,0] # obstacles
img[~world.true_map] = [1,1,1] # free space

# visited nodes
for x,y in planner.visited:
    img[x,y] = [1,0.8,0.8] #light red

# path taken
for x,y in path:
    img[x,y] = [0,0,1] # blue

img[start] = [0,1,0] # green
img[goal] = [1,0,0] # red

plt.figure(figsize=(6,6))
plt.imshow(img)
plt.title("Depth First Search (DFS)")
plt.axis('off')
plt.show()

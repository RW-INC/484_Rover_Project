import heapq
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

INF = float('inf')

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

# check to make sure there's actually a possible path
def path_exists(grid, start, goal):
    q = deque([start])
    visited = {start}
    while q:
        x, y = q.popleft()
        if (x, y) == goal: return True
        for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
            nx, ny = x+dx, y+dy
            if 0 <= nx < grid.size and 0 <= ny < grid.size:
                if not grid.is_obstacle(nx, ny) and (nx, ny) not in visited:
                    visited.add((nx, ny))
                    q.append((nx, ny))
    return False

# implementing path algorithm
# D*
class DStarLite:
    def __init__(self, size, start, goal):
        # inits
        self.size = size
        self.start = start
        self.goal = goal
        self.g = { (x, y): INF for x in range(size) for y in range(size) }
        self.rhs = { (x, y): INF for x in range(size) for y in range(size) }
        self.U = []
        self.km = 0

        self.rhs[self.goal] = 0
        heapq.heappush(self.U, (self.calculate_key(self.goal), self.goal))

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def calculate_key(self, s):
        k1 = min(self.g[s], self.rhs[s]) + self.heuristic(self.start, s) + self.km
        k2 = min(self.g[s], self.rhs[s])
        return (k1, k2)

    def neighbors(self, s):
        x, y = s
        results = []
        for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
            nx, ny = x+dx, y+dy
            if 0 <= nx < self.size and 0 <= ny < self.size:
                results.append((nx, ny))
        return results

    def cost(self, u, v, world):
        # if either is an obstacle, cost is infinite
        if world.is_obstacle(*u) or world.is_obstacle(*v):
            return INF
        return 1

    def update_vertex(self, u, world):
        if u != self.goal:
            # rhs(u) = min_{s' in nbrs} (c(u, s') + g(s'))
            costs = []
            for s_prime in self.neighbors(u):
                costs.append(self.cost(u, s_prime, world) + self.g[s_prime])
            self.rhs[u] = min(costs) if costs else INF

        # remove u from priority queue if present
        self.U = [item for item in self.U if item[1] != u]
        heapq.heapify(self.U)

        if self.g[u] != self.rhs[u]:
            heapq.heappush(self.U, (self.calculate_key(u), u))

    def compute_shortest_path(self, world):
        while self.U and (
            self.U[0][0] < self.calculate_key(self.start) or 
            self.rhs[self.start] != self.g[self.start]
        ):
            k_old, u = heapq.heappop(self.U)
            k_new = self.calculate_key(u)

            if k_old < k_new:
                heapq.heappush(self.U, (k_new, u))
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for s in self.neighbors(u):
                    self.update_vertex(s, world)
            else:
                self.g[u] = INF
                self.update_vertex(u, world)
                for s in self.neighbors(u):
                    self.update_vertex(s, world)

# main
size = 30
start_node = (0, 0)
goal_node = (size-1, size-1)

# make sure map is solvable
while True:
    world = GridWorld(size, obstacle_prob=0.2)
    if path_exists(world, start_node, goal_node):
        break

planner = DStarLite(size, start_node, goal_node)
planner.compute_shortest_path(world)

# inits for path
current = start_node
last_node = start_node
path_taken = [current] # append nodes here

while current != goal_node:
    # look at neighbors and pick the best step
    best_s = None
    min_val = INF
    
    for s_next in planner.neighbors(current):
        # move toward the neighbor that minimizes edge cost + g-value
        val = planner.cost(current, s_next, world) + planner.g[s_next]
        if val < min_val:
            min_val = val
            best_s = s_next
    
    if best_s is None or min_val == INF:
        print("No valid path found!")
        break

    # advance the path
    current = best_s
    path_taken.append(current)
    
    # update now pos in planner
    planner.km += planner.heuristic(last_node, current)
    last_node = current
    planner.start = current
    
    # Note: In a dynamic world, you would detect map changes here, 
    # call update_vertex() on changed cells, and then compute_shortest_path() again.
    # Since this model solves the map upfront it will just move without recomputing again

# plot
print(f'path length: {len(path_taken)}')
img = np.zeros((size, size, 3))
img[world.true_map] = [0, 0, 0] # black obstacles
img[~world.true_map] = [1, 1, 1] # white free

for x, y in path_taken:
    img[x, y] = [0, 0, 1] # blue path

img[start_node] = [0, 1, 0] # green start
img[goal_node] = [1, 0, 0] # red goal

plt.figure(figsize=(7, 7))
plt.imshow(img)
plt.title("D*")
plt.axis('off')
plt.show()
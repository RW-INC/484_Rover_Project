import heapq
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import time
import math
import random

INF = float('inf')

# creating map
# random nxn grid w/ obstacles
class GridWorld:
    def __init__(self, size, obstacle_prob):
        self.size = size
        # the actual world
        self.map = np.random.rand(size, size) < obstacle_prob
        self.map[0, 0] = False
        self.map[size-1, size-1] = False

    def is_obstacle(self, x, y):
        return self.map[x, y]
    
    def free(self, s):
        x,y = s
        return 0 <= x < self.size and 0 <= y < self.size and not self.map[x,y]

    def obstacle(self, s):
        return self.map[s]

    def neighbors(self, s):
        x,y = s
        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx,ny = x+dx,y+dy
            if 0 <= nx < self.size and 0 <= ny < self.size:
                if not self.map[nx,ny]:
                    yield (nx,ny)

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
# D*lite
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

# Dijkstra
class Dijkstra:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.dist = {start: 0}
        self.parent = {start: None}
        self.pq = [(0,start)]

    def neighbors(self,s):
        x,y=s
        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            n=(x+dx,y+dy)
            if self.grid.free(n):
                yield n

    def solve(self):
        while self.pq:
            d,u = heapq.heappop(self.pq)
            if u == self.goal:
                return self.reconstruct()

            for v in self.neighbors(u):
                nd = d + 1
                if v not in self.dist or nd < self.dist[v]:
                    self.dist[v] = nd
                    self.parent[v] = u
                    heapq.heappush(self.pq,(nd,v))
        return []

    def reconstruct(self):
        path=[]
        s=self.goal
        while s is not None:
            path.append(s)
            s=self.parent[s]
        return path[::-1]

# A*
class AStar:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.g = { (i,j): INF for i in range(grid.size) for j in range(grid.size) }
        self.parent = {}

    def h(self, a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1]) 

    def compute_path(self):
        open_set = []
        heapq.heappush(open_set, (0, self.start))
        self.g[self.start] = 0

        while open_set:
            _, s = heapq.heappop(open_set)

            if s == self.goal:
                return self.reconstruct_path()

            for n in self.grid.neighbors(s):
                tentative = self.g[s] + 1
                if tentative < self.g[n]:
                    self.g[n] = tentative
                    f = tentative + self.h(n, self.goal)
                    heapq.heappush(open_set, (f, n))
                    self.parent[n] = s

        return []  # no path

    def reconstruct_path(self):
        path = [self.goal]
        s = self.goal
        while s != self.start:
            s = self.parent[s]
            path.append(s)
        return path[::-1]

# RRT
class RRT:
    def __init__(self, grid, start, goal, step=1, max_iter=8000):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.step = step
        self.max_iter = max_iter
        self.tree = {start: None}
 
    def dist(self,a,b):
        return math.hypot(a[0]-b[0], a[1]-b[1])

    def sample(self):
        return (random.randint(0,self.grid.size-1),
                random.randint(0,self.grid.size-1))

    def nearest(self, s):
        return min(self.tree.keys(), key=lambda n: self.dist(n,s))

    def steer(self, a, b):
        ax, ay = a
        bx, by = b

        dx = bx - ax
        dy = by - ay

        # choose dominant axis by distance
        if abs(dx) > abs(dy):
            step = (ax + np.sign(dx), ay)
        else:
            step = (ax, ay + np.sign(dy))

        return (int(step[0]), int(step[1]))

    def collision_free(self,a,b):
        return self.grid.free(b)

    def build(self):
        for i in range(self.max_iter):
            rnd = self.sample()
            nearest = self.nearest(rnd)
            new = self.steer(nearest, rnd)

            if not self.grid.free(new):
                continue

            if new in self.tree:
                continue

            self.tree[new] = nearest

            if self.dist(new, self.goal) <= 1:
                self.tree[self.goal] = new
                return self.reconstruct_path()

        return []

    def reconstruct_path(self):
        path = [self.goal]
        s = self.goal
        while s is not None:
            s = self.tree[s]
            if s is not None:
                path.append(s)
        return path[::-1]

# DFS
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
size = 50
obstacle_prob = 0.37
trials= 1000

start_node = (0, 0)
goal_node = (size-1, size-1)

times_d = []
length_d = []
times_dijkstra = []
length_dijkstra = []
times_A = []
length_A = []
times_RRT = []
length_RRT = []
times_DFS = []
length_DFS = []
success = 0

for i in range(trials):
    if i == 100:
        print("trial 100")
    if i == 200:
        print("trial 200")
    if i == 300:
        print("trial 300")
    if i == 400:
        print("trial 400")
    if i == 500:
        print("trial 500")
    if i == 600:
        print("trial 600")
    if i == 700:
        print("trial 700")
    if i == 800:
        print("trial 800")
    if i == 900:
        print("trial 900")
    if i == 1000:
        print("trial 1000")
    
    # generate map and make sure is solvable
    while True:
        world = GridWorld(size, obstacle_prob)
        if path_exists(world, start_node, goal_node):
            break

    # timing D* lite
    start = time.time()
    planner_d = DStarLite(size, start_node, goal_node)
    planner_d.compute_shortest_path(world)

    # inits for path D* lite
    current_d = start_node
    last_node = start_node
    path_taken_d = [current_d] # append nodes here

    while current_d != goal_node:
        # look at neighbors and pick the best step
        best_s = None
        min_val = INF
        
        for s_next in planner_d.neighbors(current_d):
            # move toward the neighbor that minimizes edge cost + g-value
            val = planner_d.cost(current_d, s_next, world) + planner_d.g[s_next]
            if val < min_val:
                min_val = val
                best_s = s_next
        
        if best_s is None or min_val == INF:
            print("No valid path found!")
            break

        # advance the path
        current_d = best_s
        path_taken_d.append(current_d)
        
        # update now pos in planner
        planner_d.km += planner_d.heuristic(last_node, current_d)
        last_node = current_d
        planner_d.start = current_d

    end = time.time()
    times_d.append(end-start)
    length_d.append(len(path_taken_d))

    # timing dikstra 
    start = time.time()
    while True:
        grid = GridWorld(size,0.15)
        if path_exists(grid,start_node,goal_node): break

    planner_dijkstra = Dijkstra(grid,start_node,goal_node)
    path_dijkstra = planner_dijkstra.solve()
    end = time.time()

    times_dijkstra.append(end-start)
    length_dijkstra.append(len(path_dijkstra))

    # timing A*
    start = time.time()
    planner_A = AStar(grid,start_node,goal_node)
    path_A = planner_A.compute_path()
    end = time.time()

    times_A.append(end-start)
    length_A.append(len(path_A))

    # timing RRT
    start = time.time()
    rrt = RRT(grid,start_node,goal_node,step=1,max_iter=5000)
    path_RRT = rrt.build()
    end = time.time()

    times_RRT.append(end-start)
    length_RRT.append(len(path_RRT))

    # timing DFS
    start = time.time()
    planner_DFS = DFSPlanner(world, start_node, goal_node)
    found = planner_DFS.search()
    if not found:
        print("No path found!")
        success += 1
        exit()
    else:
        path_DFS = planner_DFS.get_path()
        end = time.time()

        times_DFS.append(end-start)
        length_DFS.append(len(path_DFS))

for i in range(len(length_RRT)):
    if i >= len(length_RRT):
        break
    if length_RRT[i] == 0:
        del length_RRT[i]
        success += 1

# print results
print(f"average times over {trials} maps (seconds)")
print(f"D* lite: {np.mean(times_d)}")
print(f"A*: {np.mean(times_A)}")
print(f"dijkstra: {np.mean(times_dijkstra)}")
print(f"RRT: {np.mean(times_RRT)}")
print(f"DFS: {np.mean(times_DFS)}\n")

print(f"average path length over {trials} maps ({size}x{size} grid)")
print(f"D* lite: {np.mean(length_d)}")
print(f"A*: {np.mean(length_A)}")
print(f"dijkstra: {np.mean(length_dijkstra)}")
print(f"RRT: {np.mean(length_RRT)}")
print(f"DFS: {np.mean(length_DFS)}\n")

print(f"unsuccessful trials: {100*success/trials}%")
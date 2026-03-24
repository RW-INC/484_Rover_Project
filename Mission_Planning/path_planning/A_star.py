import heapq
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

INF = float('inf')

# creating map
# random nxn grid w/ obstacles
# da qr code boi 
class GridWorld:
    def __init__(self, size, obstacle_prob=0.25):
        self.size = size
        self.map = np.random.rand(size,size) < obstacle_prob
        self.map[0,0] = False
        self.map[-1,-1] = False

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
# if there's no path skip that map
def path_exists(grid,start,goal):
    q=deque([start])
    vis={start}
    while q:
        s=q.popleft()
        if s==goal: return True
        for n in grid.neighbors(s):
            if n not in vis:
                vis.add(n); q.append(n)
    return False

# implementing path algorithm
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

        return []  # no path, redundancy bc soemthing isn't working frfr on god no cap

    def reconstruct_path(self):
        path = [self.goal]
        s = self.goal
        while s != self.start:
            s = self.parent[s]
            path.append(s)
        return path[::-1]

# main
size = 30
start = (0,0)
goal = (size-1,size-1)

# make sure map is solvable
while True:
    grid = GridWorld(size,0.15)
    if path_exists(grid,start,goal): break

planner = AStar(grid,start,goal)
path = planner.compute_path()

# plot
print(f"path length: {len(path)}")
img = np.zeros((size,size,3))
img[grid.map] = [0,0,0]
img[~grid.map] = [1,1,1]

for s in path:
    img[s] = [0,0,1]

img[start] = [0,1,0]
img[goal] = [1,0,0]

plt.imshow(img)
plt.title("A*")
plt.axis('off')
plt.show()

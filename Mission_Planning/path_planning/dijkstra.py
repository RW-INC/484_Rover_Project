import heapq
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

INF = float('inf')

# creating map
# random nxn grid w/ obstacles
class GridWorld:
    def __init__(self, size, p=0.3):
        self.size = size
        self.map = np.random.rand(size,size) < p
        self.map[0,0] = False
        self.map[-1,-1] = False

    def free(self, s):
        x,y = s
        return 0 <= x < self.size and 0 <= y < self.size and not self.map[x,y]

# check to make sure there's actually a possible path
def path_exists(grid,start,goal):
    q=deque([start])
    vis={start}
    while q:
        s=q.popleft()
        if s==goal: return True
        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            n=(s[0]+dx,s[1]+dy)
            if grid.free(n) and n not in vis:
                vis.add(n); q.append(n)
    return False

# implementing path algorithm
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

# main
size = 30
start=(0,0)
goal=(size-1,size-1)

while True:
    grid = GridWorld(size,0.3)
    if path_exists(grid,start,goal): break

planner = Dijkstra(grid,start,goal)
path = planner.solve()

# plot
print(f"path length: {len(path)}")
img = np.zeros((size,size,3))
img[grid.map] = [0,0,0]
img[~grid.map] = [1,1,1]

for x,y in path:
    img[x,y] = [0,0,1]

img[start] = [0,1,0]
img[goal] = [1,0,0]

plt.imshow(img)
plt.title("Dijkstra")
plt.axis('off')
plt.show()

import numpy as np
import matplotlib.pyplot as plt
import random
import math
from collections import deque

# creating map
# random nxn grid w/ obstacles
class GridWorld:
    def __init__(self, size, obstacle_prob=0.25):
        self.size = size
        self.map = np.random.rand(size,size) < obstacle_prob
        self.map[0,0] = False
        self.map[-1,-1] = False

    def obstacle(self, s):
        return self.map[s]

    def in_bounds(self, s):
        x,y = s
        return 0 <= x < self.size and 0 <= y < self.size

    def free(self, s):
        return self.in_bounds(s) and not self.map[s]

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
# RRT
class RRT:
    def __init__(self, grid, start, goal, step=1, max_iter=5000):
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

# main
size = 50
start = (0,0)
goal = (size-1,size-1)

while True:
    grid = GridWorld(size,0.25)
    if path_exists(grid,start,goal): break

rrt = RRT(grid,start,goal,step=1,max_iter=5000)
path = rrt.build()

# plot
print(f"path length: {len(path)}")
img = np.zeros((size,size,3))
img[grid.map] = [0,0,0]
img[~grid.map] = [1,1,1]

# draw tree edges
for node,parent in rrt.tree.items():
    if parent is not None:
        img[node] = [1,0.6,0.6]

# draw path
for s in path:
    img[s] = [0,0,1]

img[start] = [0,1,0]
img[goal] = [1,0,0]

plt.imshow(img)
plt.title("Rapidly-Exploring Random Trees (RRT)")
plt.axis('off')
plt.show()

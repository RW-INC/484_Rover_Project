import rasterio
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import heapq
from collections import deque
import cv2
from rasterio.windows import from_bounds # for choosing window size


# creating map
class TerrainWorld:
    def __init__(self, traversable_grid):

        self.grid = traversable_grid
        self.size = traversable_grid.shape[0]

    def is_obstacle(self, x, y): #???
        return not self.grid[x, y]
    
INF = float('inf')

# check to make sure there's actually a possible path
def path_exists(grid, start, goal):
    q = deque([start])
    visited = {start}
    while q:
        x, y = q.popleft()
        if (x, y) == goal: 
            return True
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
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
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

def old_main_ignore():
    print('hi')
    # # main
    # size = 30
    # start_node = (0, 0)
    # goal_node = (size-1, size-1)

    # # make sure map is solvable
    # while True:
    #     world = GridWorld(size, obstacle_prob=0.2)
    #     if path_exists(world, start_node, goal_node):
    #         break

    # planner = DStarLite(size, start_node, goal_node)
    # planner.compute_shortest_path(world)

    # # inits for path
    # current = start_node
    # last_node = start_node
    # path_taken = [current] # append nodes here

    # while current != goal_node:
    #     # look at neighbors and pick the best step
    #     best_s = None
    #     min_val = INF
        
    #     for s_next in planner.neighbors(current):
    #         # move toward the neighbor that minimizes edge cost + g-value
    #         val = planner.cost(current, s_next, world) + planner.g[s_next]
    #         if val < min_val:
    #             min_val = val
    #             best_s = s_next
        
    #     if best_s is None or min_val == INF:
    #         print("No valid path found!")
    #         break

    #     # advance the path
    #     current = best_s
    #     path_taken.append(current)
        
    #     # update now pos in planner
    #     planner.km += planner.heuristic(last_node, current)
    #     last_node = current
    #     planner.start = current
        
    #     # Note: In a dynamic world, you would detect map changes here, 
    #     # call update_vertex() on changed cells, and then compute_shortest_path() again.
    #     # Since this model solves the map upfront it will just move without recomputing again

    # # plot
    # print(f'path length: {len(path_taken)}')
    # img = np.zeros((size, size, 3))
    # img[world.true_map] = [0, 0, 0] # black obstacles
    # img[~world.true_map] = [1, 1, 1] # white free

    # for x, y in path_taken:
    #     img[x, y] = [0, 0, 1] # blue path

    # img[start_node] = [0, 1, 0] # green start
    # img[goal_node] = [1, 0, 0] # red goal

    # plt.figure(figsize=(7, 7))
    # plt.imshow(img)
    # plt.title("D*")
    # plt.axis('off')
    # plt.show()

def load_data(elevation_file, slope_file):
    region_size = 15000 # meters

    with rasterio.open(elevation_file) as src:
        window = from_bounds(-region_size, -region_size, region_size, region_size, src.transform)
        elevation = src.read(1, window=window)


    with rasterio.open(slope_file) as src:
        window = from_bounds(-region_size, -region_size, region_size, region_size, src.transform)
        slopes = src.read(1, window = window)

    return elevation, slopes
    
def traversability(elevation, slopes):
    # traversable terrain
    traversable = slopes < 10

    # traversable = cv2.resize(traversable.astype(np.uint8),
    #               (400,400),
    #               interpolation=cv2.INTER_NEAREST).astype(bool)

    return traversable

def find_landing_ellipses(binary_map, major_axis, minor_axis):
    major_pixels = int((major_axis * 1000)/10) # account for 10 m / pixel resolution
    minor_pixels = int((minor_axis * 1000)/10)

    a = major_pixels//2 # integer division
    b = minor_pixels//2

    h, w = binary_map.shape
    viable_sites = []

    y, x = np.ogrid[:h,:w]

    for center_y in range(b, h - b, 200): # center of landing ellipse
        current_y = (y - center_y)/b
        for center_x in range(a, w - a, 200):
            # distance from center as fraction of ellipse radius
            current_x = (x - center_x)/a 

            ellipse = (current_x**2 + current_y**2) <= 1 # points that satisfy the ellipse equation are included inside the ellipse

            region = binary_map[ellipse] # traversability inside ellipse

            trav_percent = np.sum(region)/region.size
            if trav_percent >= 0.95: # is over 95% of terrain traversable?
                viable_sites.append((center_x, center_y, trav_percent))
                print(viable_sites[-1])

    viable_sites.sort(key=lambda x: x[2], reverse=True)
    # print(viable_sites)

    top_n = min(5, len(viable_sites))
    plot_map = np.zeros_like(binary_map)

    # for i in range(top_n):
    #     cx, cy, _ = viable_sites[i]

    #     current_x = (x - cx) / a
    #     current_y = (y - cy) / b
    #     ellipse = (current_x**2 + current_y**2) <= 1

    #     plot_map[ellipse] = i + 1  # different value per rank


    cmap = matplotlib.colors.ListedColormap(['black', 'green'])
    legend = [matplotlib.patches.Patch(facecolor='green', edgecolor='black', label='Traversable (True)'),
            matplotlib.patches.Patch(facecolor='black', edgecolor='black', label='Non-Traversable (False)')]

    plt.figure(figsize=(8, 8))
    plt.title("Top 5 Viable Landing Ellipses")

    # base terrain
    plt.imshow(binary_map, cmap=cmap)

    # ellipses are overlayed in red
    ellipse_overlay = np.zeros((h, w, 4))

    for i in range(top_n):
        cx, cy, _ = viable_sites[i]

        current_x = (x - cx) / a
        current_y = (y - cy) / b
        ellipse = (current_x**2 + current_y**2) <= 1

        # Red color with transparency
        ellipse_overlay[ellipse] = [1, 0, 0, 0.35]  # R, G, B, Alpha (transparency)
        plt.contour(ellipse.astype(int), levels=[0.5], colors='red', linewidths=2)

    # Plot overlay
    plt.imshow(ellipse_overlay)

    plt.xlabel("X Pixel")
    plt.ylabel("Y Pixel")
    plt.show()

    return viable_sites

def choose_random_start_end_locations(traversable_grid, viable_site, major_axis, minor_axis):
    cx, cy, __ = viable_site
    a = int((major_axis * 1000)/10)//2
    b = int((minor_axis * 1000)/10)//2

    h, w = traversable_grid.shape
    min_distance_m = 1000
    min_distance_px = int(min_distance_m / 10)  # convert meters to pixels


    while True:
        # random point inside bounding box
        x = np.random.randint(cx-a, cx+a)
        y = np.random.randint(cy-b, cy+b)

        # check if inside ellipse
        if ((x-cx)/a)**2 + ((y-cy)/b)**2 <= 1:
            if traversable_grid[y, x]:  # traversable
                start = (y, x)
                break

    while True:
        x = np.random.randint(cx-a, cx+a)
        y = np.random.randint(cy-b, cy+b)

        if ((x-cx)/a)**2 + ((y-cy)/b)**2 <= 1:

            if traversable_grid[y, x]:
                goal = (y, x)

                dist = np.sqrt((goal[0] - start[0])**2 + (goal[1] - start[1])**2)
                if dist >= min_distance_px:
                    break

    waypoints = [start, goal]
    return waypoints

def extract_path(planner, world, start_node, goal_node):
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

    return path_taken


elevation, slopes = load_data("data/LDEM_83S_10MPP_ADJ.tiff", "data/LDSM_83S_10MPP_ADJ.tiff")
traversable_grid = traversability(elevation, slopes)

world = TerrainWorld(traversable_grid)
size = traversable_grid.shape[0]

viable_sites = find_landing_ellipses(traversable_grid, major_axis = 4, minor_axis = 2)

waypoints = choose_random_start_end_locations(traversable_grid, viable_sites[0], major_axis = 4, minor_axis = 2)

start, goal = waypoints

if path_exists(world, start, goal) == False:
    print("No valid rover path between start and goal")

full_path = []
current_start = waypoints[0]

for wp in waypoints[1:]:

    planner = DStarLite(size, current_start, wp)
    planner.compute_shortest_path(world)

    segment = extract_path(planner, world, current_start, wp)

    full_path.extend(segment)
    current_start = wp


# plotting stuff

img = np.zeros((size, size, 3))

img[world.grid] = [1,1,1] # white = traversable
img[~world.grid] = [0,0,0] # black = obstacles, not part of world.grid

for y,x in full_path:
    img[y-5:y+5, x-5:x+5] = [0,0,1] # thicker path

def mark_point(img, point, color, radius=3):
    y, x = point
    y_min = max(0, y - radius)
    y_max = min(img.shape[0], y + radius + 1)
    x_min = max(0, x - radius)
    x_max = min(img.shape[1], x + radius + 1)
    img[y_min:y_max, x_min:x_max] = color

mark_point(img, waypoints[0], [0,1,0], radius=10)   # start (green)
mark_point(img, waypoints[-1], [1,0,0], radius=10)  # goal (red)

plt.figure(figsize=(8,8))
plt.imshow(img)
plt.title("Lunar Rover Traverse (D*)")
plt.axis('off')
plt.show()

print("Path length:", len(full_path))
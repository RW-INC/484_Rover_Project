import rasterio
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import heapq
from collections import deque
import cv2
from rasterio.windows import from_bounds # for choosing window size
from scipy.io import loadmat

# python function that tells if battery is running??
# print elevations of each waypoint
# incorporate solar illumination
# produce a zoomed in plot as well
# ensure waypoints are consecutive
# label waypoints on map
# overlay waypoints on solar map and slope map



# output distance traveled by rover

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

def load_data(elevation_file, slope_file, region_size):
    # elevation data
    with rasterio.open(elevation_file) as src:
        window = from_bounds(-region_size, -region_size, region_size, region_size, src.transform)
        elevation = src.read(1, window=window)
        el_shape = elevation.shape

    # slopes data
    with rasterio.open(slope_file) as src:
        window = from_bounds(-region_size, -region_size, region_size, region_size, src.transform)
        slopes = src.read(1, window = window)

    # solar illumination data
    data = loadmat("data/illumination_data.mat")

    sun = data["all_sun_data"]
    X = data["X"]
    Y = data["Y"]

    # average sunlight across time
    avg_sun = np.mean(sun, axis=2)

    # cubic interpolation to resample solar illumination dataset to 10 m / pixel
    avg_sun_resampled = cv2.resize(avg_sun, (el_shape[1], el_shape[0]), interpolation=cv2.INTER_CUBIC)
    
    # analysis??
    # always_sunlit = np.all(sun == 1, axis=2)
    # print("always sunlit values:", always_sunlit)
    
    # num_eternal = np.sum(always_sunlit)
    # print("Cells with continuous sunlight:", num_eternal)

    # plt.imshow(avg_sun, origin="lower")
    # plt.colorbar(label="Average Sunlight (%)")
    # plt.title("Average Sunlight Over Mission")
    # plt.show()

    return elevation, slopes, avg_sun_resampled
    
def traversability(elevation, slopes):
    # traversable terrain
    traversable = slopes < 10

    # traversable = cv2.resize(traversable.astype(np.uint8),
    #               (400,400),
    #               interpolation=cv2.INTER_NEAREST).astype(bool)

    return traversable

def find_landing_ellipses(binary_map, major_axis, minor_axis):
    # convert major/minor axis length to pixels
    major_pixels = int((major_axis * 1000)/10) # account for 10 m / pixel resolution
    minor_pixels = int((minor_axis * 1000)/10)

    a = major_pixels//2 # integer division
    b = minor_pixels//2

    h, w = binary_map.shape
    viable_sites = []

    y, x = np.ogrid[:h,:w]
    counter = 1

    for cy in range(b, h - b, 200): # center of landing ellipse
        current_y = (y - cy)/b
        for cx in range(a, w - a, 200):
            # distance from center as fraction of ellipse radius
            current_x = (x - cx)/a 

            ellipse = (current_x**2 + current_y**2) <= 1 # points that satisfy the ellipse equation are included inside the ellipse

            region = binary_map[ellipse] # traversability inside ellipse

            trav_percent = np.sum(region)/region.size
            if trav_percent >= 0.95: # is over 95% of terrain traversable?
                viable_sites.append((cx, cy, float(trav_percent)))
                print(f'Viable site #{counter}: {viable_sites[-1]})')
                counter+=1

    viable_sites.sort(key=lambda x: x[2], reverse=True) # sort in descending order

    top_n = min(5, len(viable_sites))
    plot = False
    if plot == True:

        cmap = matplotlib.colors.ListedColormap(['black', 'green'])
        legend = [matplotlib.patches.Patch(facecolor='green', edgecolor='black', label='Traversable (True)'),
                matplotlib.patches.Patch(facecolor='black', edgecolor='black', label='Non-Traversable (False)')]

        plt.figure(figsize=(8, 8))
        plt.title("Top 5 Viable Landing Ellipses")

        # base terrain
        plt.imshow(binary_map, cmap=cmap)
        plt.legend(handles=legend, loc='upper right')

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

"""
Criteria for choosing waypoints:
- elevations for each waypoint are within 10 m of each other to ensure line of sight
"""
def choose_waypoints(traversable_grid, elevation, sun_avg, mission_site, major_axis, minor_axis):
    cx, cy, __ = mission_site
    a = int((major_axis * 1000)/10)//2
    b = int((minor_axis * 1000)/10)//2

    min_distance_m = 900
    min_distance_px = int(min_distance_m / 10)  # convert meters to pixels
    max_distance_px = int(2000/10)
    sun_threshold = 60 # percent
    num_payloads = 3

    waypoints = []

    while len(waypoints) < num_payloads:

        x = np.random.randint(cx-a, cx+a)
        y = np.random.randint(cy-b, cy+b)

        if ((x-cx)/a)**2 + ((y-cy)/b)**2 > 1: # inside ellipse
            continue

        if not traversable_grid[y,x]: # traversable
            continue

        if sun_avg[y,x] < sun_threshold: # sufficient sunlight
            continue

        # FIRST WAYPOINT -- the lander
        if len(waypoints) == 0: 
            start_elevation = elevation[y,x]
            waypoints.append((y,x))
            continue

        # all other waypoints should have LOS with lander
        if not line_of_sight(elevation, waypoints[0], (y,x)):
            continue

        # final waypoint must be >= 0.9 km from lander
        if len(waypoints) == num_payloads-1:  
            y0,x0 = waypoints[0]
            dist = np.sqrt((x-x0)**2 + (y-y0)**2)

            if dist < min_distance_px or dist > max_distance_px:
                continue
        
        waypoints.append((y,x))
                    
    return waypoints

def Bresenham_line_of_sight(elevation, p1, p2):
    y0, x0 = p1
    y1, x1 = p2

    z0 = elevation[y0, x0]
    z1 = elevation[y1, x1]

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)

    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1

    err = dx - dy

    steps = max(dx, dy)
    i = 0

    while True:
        if (y0, x0) != p1 and (y0, x0) != p2:

            t = i / steps
            z_expected = z0 + (z1 - z0) * t
            z_actual = elevation[y0, x0]

            if z_actual > z_expected:
                return False

        if x0 == x1 and y0 == y1:
            break

        e2 = 2 * err

        if e2 > -dy:
            err -= dy
            x0 += sx

        if e2 < dx:
            err += dx
            y0 += sy

        i += 1
    return True

def line_of_sight(elevation, point1, point2):
    y0, x0 = point1
    y1, x1 = point2

    z0 = elevation[y0, x0]
    z1 = elevation[y1,x1]

    dx = x1 - x0
    dy = y1 - y0

    step_size = int(max(abs(dx), abs(dy)))
    for i in range(1, step_size):

        t = i / step_size

        x = int(round(x0 + dx*t))
        y = int(round(y0 + dy*t))

        z_expected = z0 + (z1 - z0) * t
        z_actual = elevation[y, x]

        if z_actual > z_expected:
            return False

    return True

def path_solar_exposure(path, sun_cube, rover_speed=0.02):
    meters_per_pixel = 10
    step_time = meters_per_pixel/rover_speed

    sunlit_steps = 0

    for i, (y,x) in enumerate(path):
        t = int(i*step_time)
        t = min(t, sun_cube.shape[2]-1)

        if sun_cube[y,x,t] >= 0.99:
            sunlit_steps += 1

    percent_sun = sunlit_steps / len(path) * 100

    return percent_sun

def longest_dark_period(pixel_timeseries):
    longest = 0
    current = 0

    for s in pixel_timeseries:
        if s == 0:
            current += 1
            longest = max(longest,current)
        else:
            current = 0

    return longest

region_size = 15000 # meters
major_axis = 4 # km
minor_axis = 2

elevation, slopes, avg_sun = load_data("data/LDEM_83S_10MPP_ADJ.tiff", "data/LDSM_83S_10MPP_ADJ.tiff", region_size)
traversable_grid = traversability(elevation, slopes)

viable_sites = find_landing_ellipses(traversable_grid, major_axis, minor_axis)
mission_site = viable_sites[0]

waypoints = choose_waypoints(traversable_grid, elevation, avg_sun, mission_site, major_axis, minor_axis)

start = waypoints[0]
goal = waypoints[-1]

world = TerrainWorld(traversable_grid, mission_site)
size = traversable_grid.shape[0]

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

cx, cy, __ = mission_site

a = int((major_axis * 1000)/10)//2
b = int((minor_axis * 1000)/10)//2

h, w = traversable_grid.shape
y, x = np.ogrid[:h, :w]

ellipse = ((x - cx)/a)**2 + ((y - cy)/b)**2 <= 1

x_min = max(0, cx - a)
x_max = min(w, cx + a)

y_min = max(0, cy - b)
y_max = min(h, cy + b)

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
# plt.xlim(x_min, x_max)
# plt.ylim(y_max, y_min)   # flipped because image coordinates
plt.show()

print("Path length:", len(full_path))

################ figure 2??

sun_norm = (avg_sun - np.min(avg_sun)) / (np.max(avg_sun) - np.min(avg_sun))

# greyscale overlay with alpha
sun_gray = plt.cm.gray(sun_norm)
sun_gray[..., 3] = 0.5  # semi-transparent
fig, ax = plt.subplots(figsize=(10,10))  # create axes object

# Base map
ax.imshow(img)

# Overlay sunlight in greyscale
ax.imshow(sun_gray)

# Greyscale colorbar (attach to ax)
sm = matplotlib.cm.ScalarMappable(cmap='gray', 
                                  norm=matplotlib.colors.Normalize(vmin=np.min(sun_norm), vmax=np.max(sun_norm)))
sm.set_array([])
fig.colorbar(sm, ax=ax, label="Average Sunlight (normalized)")

# Mark waypoints
for i, wp in enumerate(waypoints):
    mark_point(img, wp, [0,0,1], radius=5)  # blue dots
    y, x = wp
    ax.text(x+5, y+5, f"WP{i+1}", color='black', fontsize=10, fontweight='bold')

# Rover path
path_y = [p[0] for p in full_path]
path_x = [p[1] for p in full_path]
ax.plot(path_x, path_y, color='blue', linewidth=2, label="Rover Path")

# Optional: mission ellipse
ax.contour(ellipse.astype(int), levels=[0.5], colors='red', linewidths=2)

ax.set_title("Lunar Rover Traverse with Sunlight Overlay (Greyscale)")
ax.axis('off')
ax.set_xlim(x_min, x_max)
ax.set_ylim(y_max, y_min)
plt.show()


########### Figure 3

# Normalize slope for display
slope_norm = (slopes - np.min(slopes)) / (np.max(slopes) - np.min(slopes) + 1e-8)  # avoid div0

# Apply colormap (use perceptually uniform for clarity)
slope_cmap = plt.cm.viridis(slope_norm)
slope_cmap[..., 3] = 0.6  # semi-transparent overlay

fig, ax = plt.subplots(figsize=(10,10))

# Base map: traversable terrain
ax.imshow(img)

# Overlay slope map
ax.imshow(slope_cmap)

# Greyscale colorbar for slope (or keep viridis colormap)
sm = matplotlib.cm.ScalarMappable(cmap='viridis', 
                                  norm=matplotlib.colors.Normalize(vmin=np.min(slopes), vmax=np.max(slopes)))
sm.set_array([])
fig.colorbar(sm, ax=ax, label="Slope (degrees)")

# Rover path
path_y = [p[0] for p in full_path]
path_x = [p[1] for p in full_path]
ax.plot(path_x, path_y, color='blue', linewidth=2, label="Rover Path")

# Waypoints
for i, wp in enumerate(waypoints):
    mark_point(img, wp, [1,0,0], radius=5) # red dots
    y, x = wp
    ax.text(x+5, y+5, f"WP{i+1}", color='white', fontsize=10, fontweight='bold')

# Optional: mission ellipse
ax.contour(ellipse.astype(int), levels=[0.5], colors='red', linewidths=2)

ax.set_title("Lunar Rover Traverse over Slope Map")
ax.axis('off')
ax.set_xlim(x_min, x_max)
ax.set_ylim(y_max, y_min)
plt.show()

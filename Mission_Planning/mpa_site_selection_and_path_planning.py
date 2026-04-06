import rasterio
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import heapq
from collections import deque
import cv2
from rasterio.windows import from_bounds # for choosing window size
from scipy.io import loadmat
import csv
import math
import time

# TODO: THINGS

# 4. maybe compare D* path with LOS considered in cost function vs without
# 5. multiply axis values by 10 so that they're in meters!
# 6. overall just reorganize the code 
# 7. add legends of green = yes black = no that type of thing
# maybe a trade study type thing of do we prioritize sunlight more or cost more
# include a run-time check in case D* is taking too long...

# heightmap of local terrain based on waypoints???

# python function that tells if battery is running??

# interactable_SOC_batt_mass.m 

# check inside cost function whether nodes at a certain time are illuminated
# then the check would be is the full path always illuminated (=1)

# creating map
class TerrainWorld:
    def __init__(self, traversable_grid):
        self.grid = traversable_grid
        self.size = traversable_grid.shape[0]

    def is_obstacle(self, x, y):
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
    def __init__(self, size, start, goal, lander, elevation, illumination):
        # inits
        self.size = size
        self.start = start
        self.goal = goal
        self.lander = lander
        self.elevation = elevation
        self.illumination = illumination
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
        
        # distance cost (diagonal vs straight)
        dx = abs(u[0] - v[0])
        dy = abs(u[1] - v[1])

        if dx == 1 and dy == 1:
            dist_cost = 1.4
        else:
            dist_cost = 1

        # LOS penalty
        if line_of_sight(self.elevation, v, self.lander):
            los_penalty = 0 # there is LOS
        else:
            los_penalty = 0.2

        solar_penalty = 0

        # y, x = v

        # # solar illumination penalty
        # if self.illumination[y,x] >= 0.5:
        #     solar_penalty = 0
        # else:
        #     solar_penalty = 00

        return dist_cost + los_penalty + solar_penalty

        # return dist_cost

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
            self.rhs[self.start] != self.g[self.start]):
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
        elevation = src.read(1, window = window)
        el_shape = elevation.shape
        transform = transform = src.window_transform(window)

    # slopes data
    with rasterio.open(slope_file) as src:
        window = from_bounds(-region_size, -region_size, region_size, region_size, src.transform)
        slopes = src.read(1, window = window)

    # solar illumination data
    data = loadmat("data/illumination_data.mat")

    sun = data["all_sun_data"]   # (ny, nx, nt)
    X = data["X"]
    Y = data["Y"]

    # crop to same region size as elevation
    x_min, x_max = -region_size, region_size
    y_min, y_max = -region_size, region_size

    x_mask = (X[0, :] >= x_min) & (X[0, :] <= x_max)
    y_mask = (Y[:, 0] >= y_min) & (Y[:, 0] <= y_max)

    sun_cropped = sun[y_mask, :, :]
    sun_cropped = sun_cropped[:, x_mask, :]

    # resample time slices to match elevation resolutions
    rows, cols = el_shape
    nt = sun_cropped.shape[2]

    sun_resampled = np.zeros((rows, cols, nt))

    for t in range(nt):
        sun_resampled[:, :, t] = cv2.resize(sun_cropped[:, :, t], (cols, rows), interpolation=cv2.INTER_NEAREST)

    # avg sunlight metric
    avg_sun = np.mean(sun_resampled, axis=2)

    row, col = 0, 0
    x, y = transform * (col, row)
    print(x, y)

    return elevation, slopes, sun_resampled, avg_sun
    
def traversability(elevation, slopes, avg_sun):
    # traversable terrain
    traversable_slopes = slopes < 10

    # solar illumination
    traversable_sun = avg_sun >= 0.75

    # combine traversable regions
    traversable = traversable_slopes & traversable_sun

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
            viable_sites.append((cx, cy, float(trav_percent)))
            # print(f'Viable site #{counter}: {viable_sites[-1]})')
            counter+=1

    viable_sites.sort(key=lambda x: x[2], reverse=True) # sort in descending order
    print(f'Viable site #1: {viable_sites[0]})')

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
        # plt.show()

    return viable_sites

"""
Criteria for choosing waypoints:
- elevations for each waypoint are within 10 m of each other to ensure line of sight
- last waypoint is at least 
"""
def choose_waypoints(traversable_grid, elevation, sun_avg, mission_site, major_axis, minor_axis):
    def dist(a, b):
        return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    cx, cy, __ = mission_site
    a = int((major_axis * 1000)/10)//2
    b = int((minor_axis * 1000)/10)//2

    min_distance_m = 9000
    max_distance_m = 10000

    min_distance_px = int(min_distance_m/10)  # convert meters to pixels
    max_distance_px = int(max_distance_m/10)
    sun_threshold = 60 # percent
    stop_locations = 3 # number of payloads / drop-off locations

    max_attempts = 10
    attempts = 1000

    for attempt in range(max_attempts):
        while_counter = 0
        waypoints = []
        last_dist = 0

        # choose start
        for __ in range(attempts):
            x = np.random.randint(cx-a, cx+a)
            y = np.random.randint(cy-b, cy+b)

            if ((x-cx)/a)**2 + ((y-cy)/b)**2 > 1:
                continue
            if not traversable_grid[y,x]:
                continue
            if sun_avg[y,x] < sun_threshold:
                continue

            start = (y,x)

            # choose end based on ideal distance between waypoints
            for __ in range(attempts):
                x = np.random.randint(cx-a, cx+a)
                y = np.random.randint(cy-b, cy+b)

                end = (y,x)
                distance = dist(start, end)

                if distance < min_distance_px or distance > max_distance_px:
                    continue
                if not traversable_grid[y,x]:
                    continue
                if sun_avg[y,x] < sun_threshold:
                    continue
                if not line_of_sight(elevation, start, end):
                    continue
                break
            else: 
                print('Could not find viable end. Restarting waypoint selection...')
                continue # restart outer loop

            waypoints.append(start)
            break
        else: 
            print('Could not find viable start. Restarting waypoint selection...')
            continue # restart outer loop

        

        # choose intermediate waypoints progressively farther from start 
        for attempt in range(max_attempts):
            while len(waypoints) < stop_locations - 1:
                # prevent infinite loop
                while_counter += 1
                if while_counter >= 100:
                    print('Could not find viable random waypoints based on chosen start location. Restarting waypoint selection...')
                    break

                x = np.random.randint(cx-a, cx+a)
                y = np.random.randint(cy-b, cy+b)
                current_point = (y,x)

                if not traversable_grid[y,x]:
                    continue
                if sun_avg[y,x] < sun_threshold:
                    continue
                if not line_of_sight(elevation, start, current_point):
                    continue

                distance = dist(start, current_point)

                # must be farther than last waypoint but closer than end
                if last_dist < distance < dist(start, end):
                    last_dist = distance
                    waypoints.append(current_point)

                if len(waypoints) == stop_locations - 1:
                    waypoints.append(end)
                    return waypoints
            
    if len(waypoints) < stop_locations:
        print('Failed to find viable waypoints.')
                    
    return None

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

def axis_limits(full_path):
    full_path = np.array(full_path)

    y = full_path[:,0]
    x = full_path[:,1]

    buffer = 15
    
    x_min = int(x.min() - buffer)
    x_max = int(x.max() + buffer)
    y_min = int(y.min() - buffer)
    y_max = int(y.max() + buffer)

    # make sure limits are not negative
    x_min = max(0, x_min)
    y_min = max(0, y_min)

    return x_min, x_max, y_min, y_max

def path_traversal_hillshade(world, full_path, waypoints, x_min, x_max, y_min, y_max):
    size = world.grid.shape[0]
    img = np.zeros((size, size, 3))

    img[world.grid] = [1,1,1] # white = traversable
    img[~world.grid] = [0,0,0] # black = obstacles, not part of world.grid

    for y,x in full_path:
        img[y-1:y+1, x-1:x+1] = [0,0,1] # thicker path

    __, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(img)

    # show start and end point
    y0, x0 = waypoints[0]
    circle = plt.Circle((x0, y0), radius=1, color='green', fill=True)
    ax.add_patch(circle)
    ax.text(x0 + 5, y0 + 5, f"Start", color='green', fontsize=10)

    yf, xf = waypoints[-1]
    circle = plt.Circle((xf, yf), radius=1, color='red', fill=True)
    ax.add_patch(circle)
    ax.text(xf + 5, yf + 5, f"End", color='red', fontsize=10)

    total_distance = calculate_path_distance(full_path)

    ax.set_title(f"Lunar Rover Traverse (D*) - {total_distance:.3f} m Path")
    # ax.axis('off')
    plt.xlim(x_min, x_max)
    plt.ylim(y_max, y_min)   # flipped because image coordinates
    plt.show()

def plot_LOS_path(world, elevation, full_path, waypoints, x_min, x_max, y_min, y_max):
    size = world.grid.shape[0]

    fig, ax = plt.subplots(figsize=(10, 10))

    # base layer - elevation heatmap 
    local_elevation = elevation[y_min:y_max, x_min:x_max]
    vmin = np.min(local_elevation)
    vmax = np.max(local_elevation)

    im = ax.imshow(elevation, cmap='terrain', vmin=vmin, vmax=vmax)
    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label("Elevation")

    # draw path and check LOS between point and lander
    for i in range(len(full_path) - 1):
        p1 = full_path[i]
        p2 = full_path[i+1]

        # check LOS between node and lander
        if line_of_sight(elevation, p1, waypoints[0]):
            color = 'green'
        else:
            color = 'black'

        ax.plot([p1[1], p2[1]], [p1[0], p2[0]], color=color, linewidth=2)

    # draw waypoints
    for i, wp in enumerate(waypoints):
        y, x = wp
        circle = plt.Circle((x, y), radius=1, color='blue', fill=True)
        ax.add_patch(circle)
        ax.text(x + 5, y + 5, f"WP{i+1}", color='blue', fontsize=10)

    ax.set_title("D* Path - Line of Sight")
    # ax.axis('off')
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_max, y_min)
    # ax.set_xlim(0, elevation.shape[1])
    # ax.set_ylim(elevation.shape[0], 0)
    plt.show()

"""
Calculates whether the rover is in sunlight or shade at the time t it is traversing a certain
pixel on the pre-determined D* path. Based on the rover travelling 2 cm/s, or 10/0.02 = 500 seconds to traverse each pixel.
"""

def plot_illumination_path(sun_data, avg_sun, full_path, waypoints, x_min, x_max, y_min, y_max):

    fig, ax = plt.subplots(figsize=(10, 10))

    # --- Base layer: solar illumination heatmap ---
    im = ax.imshow(avg_sun, cmap='inferno')
    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label("Average Sunlight (%)")

    rover_speed = 0.02  # m/s
    meters_per_pixel = 10
    time_per_pixel = meters_per_pixel / rover_speed  # 500 sec per pixel
    sun_window_time = 10 * 3600  # 10 hours in seconds

    total_time = 0

    # --- Overlay path colored by sunlight at time t ---
    for i in range(len(full_path) - 1):
        p1 = full_path[i]
        p2 = full_path[i+1]

        total_time += time_per_pixel

        sun_idx = int(total_time / sun_window_time)
        sun_idx = min(sun_idx, sun_data.shape[2] - 1)

        y, x = p1

        if sun_data[y, x, sun_idx] >= 90:
            color = 'lime'
        else:
            color = 'black'

        ax.plot([p1[1], p2[1]], [p1[0], p2[0]], color=color, linewidth=2)

    # --- Draw waypoints ---
    for i, wp in enumerate(waypoints):
        y, x = wp
        circle = plt.Circle((x, y), radius=1, color='blue', fill=True)
        ax.add_patch(circle)
        ax.text(x + 5, y + 5, f"WP{i+1}", color='blue')

    # --- Zoom into landing ellipse region ---
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_max, y_min)  # IMPORTANT: invert y-axis for image coords

    ax.set_title("D* Path Over Solar Illumination Map\nGreen = Sunlight, Black = Shadow")
    plt.show()

def store_path_results(full_path, waypoints):
    total_distance = calculate_path_distance(full_path)

    csv_file_path = f"planned_path_nodes_meters_{total_distance:.2f}.csv"

    with open(csv_file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        
        # write header
        writer.writerow(["x", "y"])
        
        # write each part of path
        for node in full_path:
            writer.writerow([node[1], node[0]])

        total_distance = calculate_path_distance(full_path)
    
    print(f'Total distance traversed: {total_distance} m')
    
    print(f"The path has been stored successfully inside '{csv_file_path}'.")

def calculate_path_distance(full_path):
    total_distance = 0

    for i in range(len(full_path) - 1):
        dx = abs(full_path[i+1][0] - full_path[i][0])
        dy = abs(full_path[i+1][1] - full_path[i][1])

        if dx == 1 and dy == 1: # diagonal movement
            total_distance += 10 * math.sqrt(2)
        elif dx == 1 and dy == 0: # straight movement
            total_distance += 10
        elif dx == 0 and dy == 1: # straight movement
            total_distance += 10

    return total_distance

def read_path_results():
    csv_file_path = "planned_path_nodes_70.0.csv"

    with open(csv_file_path, mode='r') as file:
        reader = csv.reader(file)
        full_path_x = []
        full_path_y = []
        
        # skip first two lines
        next(reader)
        next(reader)
        
        # read each waypoint
        for row in reader:
            full_path_x.append(float(row[0]))
            full_path_y.append(float(row[1]))
    
    print(f"The path has been read successfully inside '{csv_file_path}'.")

    full_path = np.array([full_path_x, full_path_y]).T
    return full_path


region_size = 15000 # meters
major_axis = 10 # km
minor_axis = 5

elevation, slopes, sun_data, avg_sun = load_data("data/LDEM_83S_10MPP_ADJ.tiff", "data/LDSM_83S_10MPP_ADJ.tiff", region_size)
traversable_grid = traversability(elevation, slopes, avg_sun)

viable_sites = find_landing_ellipses(traversable_grid, major_axis, minor_axis)
mission_site = viable_sites[0]

waypoints = choose_waypoints(traversable_grid, elevation, avg_sun, mission_site, major_axis, minor_axis)

print(waypoints)

# or you can manually input waypoints
# waypoints = [(344, 523), (214, 302), (247, 429)]

start = waypoints[0]
goal = waypoints[-1]

world = TerrainWorld(traversable_grid)
size = traversable_grid.shape[0]

for i in range(0, len(waypoints) - 1):
    if path_exists(world, waypoints[i], waypoints[i+1]) == False:
        print("No valid rover path between start and goal. Choosing new waypoints...")
        waypoints = choose_waypoints(traversable_grid, elevation, avg_sun, mission_site, major_axis, minor_axis)
        start = waypoints[0]
        goal = waypoints[-1]

full_path = []
current_start = start

print('Planning path based on D star algorithm...')
for wp in waypoints[1:]:
    planner = DStarLite(size, current_start, wp, start, elevation, avg_sun)
    planner.compute_shortest_path(world)

    segment = extract_path(planner, world, current_start, wp)

    if full_path:
        full_path.extend(segment[1:])
    else:
        full_path.extend(segment)
    current_start = wp

x_min, x_max, y_min, y_max = axis_limits(full_path)

path_traversal_hillshade(world, full_path, waypoints, x_min, x_max, y_min, y_max)

plot_LOS_path(world, elevation, full_path, waypoints, x_min, x_max, y_min, y_max)

plot_illumination_path(sun_data, avg_sun, full_path, waypoints, x_min, x_max, y_min, y_max)

store_path_results(full_path, waypoints)

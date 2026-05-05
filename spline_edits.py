import pygame
import sys
import numpy as np
from scipy.interpolate import splprep, splev, BSpline, PPoly
from pygame._sdl2 import Window
import ctypes
ctypes.windll.user32.SetProcessDPIAware()

pygame.init()

# --- SCALE ---
(SCREEN_W, SCREEN_H) = pygame.display.get_desktop_sizes()[0]
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H), pygame.RESIZABLE | pygame.SCALED | pygame.NOFRAME)
WIDTH_FT = 3.048 # THIS IS ACTUALLY IN METERS BUT I DONT WANT TO CHANGE ALL OF THE VARIABLE NAMES
HEIGHT_FT = 1.8288 # THIS IS ACTUALLY IN METERS BUT I DONT WANT TO CHANGE ALL OF THE VARIABLE NAMES
SCALE = 100
WIDTH, HEIGHT = WIDTH_FT * SCALE, HEIGHT_FT * SCALE
CENTER_FT = (WIDTH_FT/2, HEIGHT_FT/2)
prev_loc = CENTER_FT

screen = pygame.display.set_mode((SCREEN_W, SCREEN_H), pygame.FULLSCREEN)

# Your working "virtual canvas"
WORK_W, WORK_H = WIDTH_FT*SCALE, HEIGHT_FT*SCALE

# Center it on screen
offset_x = (SCREEN_W - WORK_W) // 2
offset_y = (SCREEN_H - WORK_H) // 2
pygame.display.set_caption("Spline Path Drawer")

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
GRID_COLOR = (50, 50, 50)

draw_surface = pygame.Surface((WORK_W, WORK_H))
draw_surface.fill(BLACK)

fade_surface = pygame.Surface((WORK_W, WORK_H))
fade_surface.fill(BLACK)
fade_alpha = 25
starting_opac = 1
opac_range = 0.6
blink_opacity = starting_opac

drawing = False
last_pos = None
pen_radius = 5

ripples = []   # each ripple: [radius, alpha]

# --- STORE PATH ---
raw_points = []
latest_spline_pts = None
status = "WAITING"

clock = pygame.time.Clock()

def draw_grid(surface):
    for x in range(0, WIDTH + 1, SCALE):
        pygame.draw.line(surface, GRID_COLOR, (x, 0), (x, HEIGHT), 1)
    for y in range(0, HEIGHT + 1, SCALE):
        pygame.draw.line(surface, GRID_COLOR, (0, y), (WIDTH, y), 1)

def pixel_to_feet(pos): # substitution which is more robust than before
    x, y = pos

    # Pygame canvas corners in pixels
    src = np.array([
        [0, 0],              # top-left
        [WORK_W, 0],         # top-right
        [WORK_W, WORK_H],    # bottom-right
        [0, WORK_H]          # bottom-left
    ], dtype=float)

    # Replace with our measurements!!!!
    dst = np.array([
        [-5.4,  3.0],   # top-left
        [ 5.4,  3.0],   # top-right
        [ 4.6, -3.0],   # bottom-right
        [-4.6, -3.0]    # bottom-left
    ], dtype=float)

    # Solve homography H
    A = []
    for (xp, yp), (X, Y) in zip(src, dst):
        A.append([xp, yp, 1, 0, 0, 0, -X*xp, -X*yp, -X])
        A.append([0, 0, 0, xp, yp, 1, -Y*xp, -Y*yp, -Y])

    A = np.array(A, dtype=float)

    _, _, Vt = np.linalg.svd(A)
    H = Vt[-1].reshape(3, 3)
    H = H / H[2, 2]

    # Apply homography
    p = np.array([x, y, 1.0])
    q = H @ p
    q = q / q[2]

    return (q[0], q[1])

def downsample(points, step=5):
    return points[::step]

def fit_spline(points):
    # if len(points) < 2:
    #     return None
    print(points)
    pts = np.array(points)
    x = pts[:, 0]
    y = pts[:, 1]

    # Choose spline degree based on number of points
    k = min(3, len(points) - 1)

    try:
        tck, u = splprep([x, y], s=0.5, k=k)
        u_new = np.linspace(0, 1, 100)
        x_s, y_s = splev(u_new, tck)
        return list(zip(x_s, y_s)), tck

    except Exception as e:
        print("Spline fit failed:", e)
        return None

def draw_spline(pts, color = GREEN):
    for i in range(len(pts) - 1):
        x1, y1 = pts[i]
        x2, y2 = pts[i + 1]

        p1 = (int(x1 * SCALE), int(y1 * SCALE))
        p2 = (int(x2 * SCALE), int(y2 * SCALE))

        pygame.draw.line(draw_surface, color, p1, p2, 3)

def generate_return_spline(start, end):
    x0, y0 = start
    x1, y1 = end

    # Create intermediate control points for smoothness
    mid_x = (x0 + x1) / 2
    mid_y = (y0 + y1) / 2

    # Slight curve offset (perpendicular direction)
    dx = x1 - x0
    dy = y1 - y0
    length = max((dx**2 + dy**2)**0.5, 1e-6)

    # perpendicular vector
    px = -dy / length
    py = dx / length

    curve_strength = 0.5  # tweak this
    ctrl_x = mid_x + px * curve_strength
    ctrl_y = mid_y + py * curve_strength

    pts = [
        (x0, y0),
        (ctrl_x, ctrl_y),
        (x1, y1)
    ]

    result = fit_spline(pts)
    return result

def tck_to_ppoly(tck):
    t, c, k = tck
    c = np.asarray(c)

    # --- scalar spline ---
    if c.ndim == 1:
        spl = BSpline(t, c, k)
        return PPoly.from_spline(spl)

    # --- parametric spline (x(u), y(u), ...) ---
    polys = []
    for dim in range(c.shape[0]):
        spl = BSpline(t, c[dim], k)
        polys.append(PPoly.from_spline(spl))

    return polys

def get_coeffs(coeffs):
    if len(coeffs) == 4:
        return coeffs
    if len(coeffs) == 3:
        b, c, d = coeffs
        return (0, b, c, d)
    if len(coeffs) == 2:
        c, d = coeffs
        return (0, 0, c, d)
    if len(coeffs) == 1:
        return (0, 0, 0, coeffs)
    if len(coeffs) == 0:
        return (0, 0, 0, 0)

def print_ppoly(pp, name="f"):
    breaks = pp.x      # knot breakpoints
    coeffs = pp.c      # shape: (deg+1, n_intervals)

    n_intervals = coeffs.shape[1]

    for i in range(n_intervals):
        if breaks[i] != breaks [i+1]:
            a, b, c, d = get_coeffs(coeffs[:, i])
            print(f"\nInterval [{breaks[i]:.4f}, {breaks[i+1]:.4f}]:")
            print(
                f"{name}(u) = "
                f"{a:.6g}u^3 + {b:.6g}u^2 + {c:.6g}u + {d:.6g}"
            )

def generate_polynomial_array(polys):
    # polys = [PPoly_x, PPoly_y]

    pp_x, pp_y = polys

    coeffs_x = pp_x.c   # shape (4, n_intervals)
    coeffs_y = pp_y.c

    breaks = pp_x.x  # right endpoints of intervals

    rows = []

    n_intervals = coeffs_x.shape[1]

    for i in range(n_intervals):
        if breaks[i] == breaks[i+1]:
            pass
        else:
            row = np.concatenate([
                get_coeffs(coeffs_x[:, i]),   # x: [a3, a2, a1, a0]
                get_coeffs(coeffs_y[:, i]),   # y: [a3, a2, a1, a0]
                [breaks[i+1]]       # breakpoint
            ])
            rows.append(row)

    return np.vstack(rows)

t = 0
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                pygame.mouse.set_pos((offset_x + prev_loc[0]*SCALE, offset_y + prev_loc[1]*SCALE))
                drawing = True
                raw_points = []          # reset path
                mx, my = event.pos
                last_pos = (mx - offset_x, my - offset_y)     # <-- FIX: initialize here

        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                drawing = False
                blink_opacity = starting_opac
                print(raw_points)
                if len(raw_points) > 1:
                    status = "DRAWN"
                    pts_ds = downsample(raw_points, step=1)
                    result = fit_spline(pts_ds)

                    if result is not None:
                        spline_pts, tck = result
                        latest_spline_pts = spline_pts  # STORE IT

        elif event.type == pygame.MOUSEMOTION:
            if drawing:
                mx, my = event.pos
                pos = (mx - offset_x, my - offset_y)

                # Only draw if inside working area
                if not (0 <= pos[0] < WORK_W and 0 <= pos[1] < WORK_H):
                    continue

                # Draw
                if last_pos:
                    pygame.draw.line(draw_surface, WHITE, last_pos, pos, pen_radius * 2)

                last_pos = pos

                # Store in FEET
                raw_points.append(pixel_to_feet(pos))

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_s:
                # print("\n--- SPLINE POINTS (feet) ---")
                # for p in spline_pts:
                #     print(f"{p[0]:.3f}, {p[1]:.3f}")

                print("\n(Control structure tck for advanced use)")
                print(tck)
                polys = tck_to_ppoly(tck)
                print(len(polys))
                names = ["x", "y"]
                for i in range(2):
                    print_ppoly(polys[i], names[i])
                polynomial_array = generate_polynomial_array(polys)
                print(polynomial_array)

            elif event.key == pygame.K_d:
                if latest_spline_pts is not None:
                    # Convert feet → pixels and draw
                    draw_spline(latest_spline_pts)
                    status = "CONFIRMED"
                    prev_loc = latest_spline_pts[-1]

            elif event.key == pygame.K_c:
                return_spline = generate_return_spline(prev_loc, (WIDTH_FT/2, HEIGHT_FT/2))
                if return_spline is not None:
                    spline_pts, tck = return_spline
                    latest_spline_pts = spline_pts

                    draw_spline(latest_spline_pts, BLUE)  # visually distinct
                    status = "CONFIRMED"
                    prev_loc = latest_spline_pts[-1]

            elif event.key == pygame.K_q and (pygame.key.get_mods() & pygame.KMOD_CTRL):
                running = False

    # Fade effect
    fade_surface.set_alpha(fade_alpha)
    draw_surface.blit(fade_surface, (0, 0))

    screen.fill(BLACK)

    # Blit your working area into center
    screen.blit(draw_surface, (offset_x, offset_y))

    # Draw grid ON TOP of working area
    grid_surface = pygame.Surface((WORK_W, WORK_H), pygame.SRCALPHA)
    draw_grid(grid_surface)
    screen.blit(grid_surface, (offset_x, offset_y))
    pygame.draw.rect(
        screen,
        (255, 0, 0),
        (offset_x, offset_y, WORK_W, WORK_H),
        2
    )

    if len(ripples) == 0 or ripples[-1][0] > 15:
        ripples.append([0, 255])  # start small, fully visible

    if status == "CONFIRMED":
        x, y = latest_spline_pts[-1]
    elif status == "DRAWN":
        x, y = prev_loc
        op = 255*blink_opacity
        draw_spline(latest_spline_pts, (op, op, op))
    elif status == "WAITING":
        x, y = CENTER_FT


    px = int(x * SCALE)
    py = int(y * SCALE)

    # Surface for transparency
    ripple_surface = pygame.Surface((WORK_W, WORK_H), pygame.SRCALPHA)

    new_ripples = []
    for r, a in ripples:
        r += .4 # expand speed
        a -= 2 # fade speed

        if a > 0:
            pygame.draw.circle(
                ripple_surface,
                (0, 255, 0, int(a)), (px, py), int(r), 2)
            new_ripples.append([r, a])

    ripples = new_ripples

    # Draw all ripples at once
    screen.blit(ripple_surface, (offset_x, offset_y))

    # Draw solid center point (rover)
    pygame.draw.circle(screen, (0, 255, 0), (px+offset_x, py+offset_y), 5)


    pygame.display.flip()
    t += 0.1
    
    blink_opacity = (np.sin(t)*opac_range/2)+(1-opac_range/2)
    clock.tick(60)

pygame.quit()
sys.exit()
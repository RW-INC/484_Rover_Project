% =========================================================================
% BUILD_SUN_MAP.m
%
% Generates a binary illumination matrix:
%   sun_map(path_step, time_step) = 1 if the rover is in sunlight, 0 if not
%
% Rows  = path nodes  (from planned_path_nodes.csv, south-pole-origin metres)
% Cols  = mission time steps (matching SOC code: t = 0:dt:T_total, dt=0.01 hr)
%
% Saves sun_map.mat for use by interactable_SOC_batt_mass.m
% =========================================================================

clear; clc;

% -------------------------------------------------------------------------
%% 1. LOAD TERRAIN
% -------------------------------------------------------------------------

load('south_pole_elevation.mat')   % provides X (km), Y (km), elevation (km)

% DEM is 100 m / pixel because it was scaled x10 from the original 10 m DEM
% Grid spacing in km:
dx_km = X(1,2) - X(1,1);          % should be ~0.1 km  (100 m)
fprintf('DEM grid spacing: %.4f km (%.1f m)\n', dx_km, dx_km*1000);

% -------------------------------------------------------------------------
%% 2. CURVATURE CORRECTION  (same as gif code)
% -------------------------------------------------------------------------

R_moon = 1737.4;                   % km
Z = elevation - (X.^2 + Y.^2) / (2 * R_moon);

% -------------------------------------------------------------------------
%% 3. TERRAIN GRADIENT  (units: km/km = dimensionless, correct for shadow)
% -------------------------------------------------------------------------

[dzdx, dzdy] = gradient(Z, dx_km);

% -------------------------------------------------------------------------
%% 4. LOAD PLANNED PATH
%
%    CSV layout (2-column, top-left origin):
%      header row: x, y
%      data rows:  x_metres_from_topleft,  y_metres_from_topleft
%
%    Origin convention: (0,0) = top-left corner of the DEM grid.
%      x increases eastward, y increases DOWNWARD (image convention).
%
%    The DEM X/Y axes are centred on the south pole (origin = centre),
%    with Y increasing NORTHWARD (geographic convention).
%
%    Conversion (grid dimensions come from the loaded DEM):
%      grid_width_m  = number of DEM columns * 100 m/pixel
%      grid_height_m = number of DEM rows    * 100 m/pixel
%
%      x_south_pole_km = (x_tl_m - grid_width_m/2)  / 1000
%      y_south_pole_km = (grid_height_m/2 - y_tl_m) / 1000   <- flip y
% -------------------------------------------------------------------------

path_data = readmatrix('planned_path_nodes_meters (2).csv', ...
    'NumHeaderLines', 1);          % skip the "x,y" header row

% path_data columns: [x, y]
%
% Coordinate convention in this CSV:
%   The path planner ran on the original 10 m/pixel DEM (5000x5000 px,
%   covering -25000 m to +25000 m).  The CSV values are pixel indices in
%   that original grid (NOT metres, despite the filename).
%
%   Origin = top-left pixel (0,0) = (-25000 m, -25000 m geographic).
%   x increases eastward,  y increases DOWNWARD (image convention).
%
% Conversion to south-pole-centred km  (verified against DEM generation code):
%   x_km = (csv_x * 10 - 25000) / 1000    <- each pixel = 10 m east of -25 km
%   y_km = (25000 - csv_y * 10) / 1000    <- flip: y=0 -> +25 km north

region_m = 25000;   % half-width of DEM in metres (matches region_length in Python)

x_raw = path_data(:, 1);   % already in metres
y_raw = path_data(:, 2);   % already in metres

path_x_km = (x_raw - region_m) / 1000;   % east,  km
path_y_km = (region_m - y_raw) / 1000;   % north, km (y flipped)

n_path = length(path_x_km);
fprintf('Loaded %d path nodes.\n', n_path);
fprintf('Path x range: %.2f to %.2f km\n', min(path_x_km), max(path_x_km));
fprintf('Path y range: %.2f to %.2f km\n', min(path_y_km), max(path_y_km));

% -------------------------------------------------------------------------
%% 5. MAP EACH PATH NODE TO THE NEAREST DEM GRID CELL
% -------------------------------------------------------------------------

X_vec = X(1, :);   % 1-D east  axis of DEM
Y_vec = Y(:, 1);   % 1-D north axis of DEM

% For each node find the closest grid index
col_idx = zeros(n_path, 1);   % column index into DEM  (east  → X)
row_idx = zeros(n_path, 1);   % row    index into DEM  (north → Y)

for k = 1:n_path
    [~, col_idx(k)] = min(abs(X_vec - path_x_km(k)));
    [~, row_idx(k)] = min(abs(Y_vec - path_y_km(k)));
end

% Clamp to valid range just in case a node lands slightly outside the DEM
col_idx = max(1, min(size(Z, 2), col_idx));
row_idx = max(1, min(size(Z, 1), row_idx));

% -------------------------------------------------------------------------
%% 6. TIME VECTOR  — must match interactable_SOC_batt_mass.m exactly
% -------------------------------------------------------------------------

n_epoch = 2836;          % Oct 1, 2027  (same as SOC code uses as n=2831)
T_total  = 24 * 14;      % hours — 14-day mission window
dt       = 0.01;         % hours (~36 sec)  — same as SOC code
t        = 0:dt:T_total;
N        = length(t);

fprintf('Time vector: %d steps  (dt = %.4f hr)\n', N, dt);

% -------------------------------------------------------------------------
%% 7. SOLAR MODEL CONSTANTS  (same as both existing codes)
% -------------------------------------------------------------------------

tau       = deg2rad(-1.545);
Y_period  = 346.71;
t0        = -1.23;
L         = deg2rad(-90);          % south pole latitude
T_lunar   = 708.75;                % hours per lunar day
omega     = 360 / T_lunar;

% -------------------------------------------------------------------------
%% 8. BUILD THE SUN MAP
%
%    sun_map(p, i) = 1  → path node p is in sunlight at mission time t(i)
%                  = 0  → in shadow
%
%    Illumination test (gradient method, same as gif code):
%      A point is in shadow when the terrain "faces away" from the sun,
%      i.e. the projected terrain slope in the sun direction exceeds tan(beta).
%      This is a local-horizon / self-shadowing check; it does NOT account
%      for cast shadows from distant peaks.  For a fuller cast-shadow model
%      you would ray-march along the sun direction — add that here later if
%      needed.
% -------------------------------------------------------------------------

fprintf('Building sun_map (%d path nodes x %d time steps)...\n', n_path, N);

sun_map = false(n_path, N);        % preallocate as logical to save memory

t_lunar_offset = (n_epoch - 2831) * 24;   % = 0 for Oct 1 2027

tic
for i = 1:N

    ti = t(i);

    % --- solar declination ---
    delta = tau * sin(deg2rad((360 / Y_period) * (n_epoch + ti/24 - t0)));

    % --- hour angle ---
    H = deg2rad(omega * (T_lunar/2 - (ti + t_lunar_offset)));

    % --- solar elevation angle ---
    beta = asin(cos(L)*cos(delta)*cos(H) + sin(L)*sin(delta));

    % Below horizon → entire surface in shadow
    if beta <= 0
        % sun_map(:, i) already false
        continue
    end

    % --- solar azimuth ---
    phi_s = atan2( ...
        cos(delta)*sin(H), ...
        sin(L)*cos(delta)*cos(H) - cos(L)*sin(delta) ...
    );

    sx = cos(beta) * cos(phi_s);
    sy = cos(beta) * sin(phi_s);

    % --- gradient shadow test at each path node ---
    for p = 1:n_path
        r = row_idx(p);
        c = col_idx(p);

        terrain_slope_proj = dzdx(r,c)*sx + dzdy(r,c)*sy;

        % In sun if local slope in sun direction does NOT exceed tan(beta)
        if terrain_slope_proj <= tan(beta)
            sun_map(p, i) = true;
        end
    end

    % Progress every 10 %
    if mod(i, round(N/10)) == 0
        fprintf('  %.0f %% done  (%.1f s elapsed)\n', 100*i/N, toc);
    end
end

fprintf('Done.  Total time: %.1f s\n', toc);

% -------------------------------------------------------------------------
%% 9. QUICK SANITY PLOT
%    X-axis = cumulative distance along path (km), the true 1-D ordering.
%    Using east position alone causes duplicate x values wherever the path
%    curves north/south, which is what caused the scattered-looking plot.
% -------------------------------------------------------------------------

sun_fraction = mean(sun_map, 2);   % average over time for each node

% Cumulative distance along path
dists    = sqrt(diff(path_x_km).^2 + diff(path_y_km).^2);
cum_dist = [0; cumsum(dists)];     % n_path x 1, in km

% Plot 1: illumination vs distance along path
figure;
plot(cum_dist, sun_fraction * 100, 'b.-', 'MarkerSize', 8, 'LineWidth', 1)
xlabel('Distance along path (km)')
ylabel('% time in sunlight')
title('Illumination fraction per path node')
ylim([0 105])
grid on

% Plot 2: path coloured by illumination fraction on DEM footprint
figure;
scatter(path_x_km, path_y_km, 30, sun_fraction * 100, 'filled')
colorbar
colormap(parula)
caxis([0 100])
xlabel('East (km)')
ylabel('North (km)')
title('Path coloured by % time in sunlight')
axis equal
grid on

% -------------------------------------------------------------------------
%% 10. SAVE
% -------------------------------------------------------------------------

save('sun_map.mat', 'sun_map', 'path_x_km', 'path_y_km', 't', 'dt', ...
     'n_path', 'N', '-v7.3');   % v7.3 supports >2 GB files if needed

fprintf('\nsun_map.mat saved.\n');
fprintf('Matrix size: %d path nodes  x  %d time steps\n', n_path, N);
fprintf('Memory footprint (logical): %.1f MB\n', n_path*N / 8 / 1e6);
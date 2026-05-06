% =========================================================================
% BUILD_SUN_MAP.m
%
% Generates a 1,0 illumination matrix:
%   sun_map(path_step, time_step) = 1 if the rover is in sunlight, 0 if not
%
% Rows  = path nodes  (from planned path CSV)
% Cols  = mission time steps (matching SOC code: t = 0:dt:T_total)
%
%  SHADOW METHOD: ray marching
%   A point P is in shadow if any terrain between P and the sun
%   rises above the line of sight: zb > zp + d * tan(beta)
%
% Saves sun_map.mat for use by interactable_SOC_batt_mass_w_path.m
% =========================================================================

clear; clc;

%% LOAD TERRAIN


load('south_pole_elevation.mat')   % X (km), Y (km), elevation (km)

dx = X(1,2) - X(1,1);
fprintf('DEM grid spacing: %.4f km (%.1f m)\n', dx, dx*1000);

x_vec = X(1,:);   % 1-D east  axis (km)
y_vec = Y(:,1);   % 1-D north axis (km)


%% CURVATURE CORRECTION

R_moon = 1737.4;   % km
Z = elevation - (X.^2 + Y.^2) / (2 * R_moon);


%% LOAD PLANNED PATH

path_data = readmatrix('Dstar_path_6221_METERS.csv', ...
    'NumHeaderLines', 1);

region_m = 25000;   % half-width of DEM in metres

x_raw = path_data(:, 1);
y_raw = path_data(:, 2);

path_x_km = (x_raw - region_m) / 1000;   % east,  km
path_y_km = (region_m - y_raw) / 1000;   % north, km (y flipped)

n_path = length(path_x_km);
fprintf('Loaded %d path nodes.\n', n_path);
fprintf('Path x range: %.2f to %.2f km\n', min(path_x_km), max(path_x_km));
fprintf('Path y range: %.2f to %.2f km\n', min(path_y_km), max(path_y_km));

%% GET ELEVATION AT EACH PATH NODE


col_idx = zeros(n_path, 1);
row_idx = zeros(n_path, 1);

for k = 1:n_path
    [~, col_idx(k)] = min(abs(x_vec - path_x_km(k)));
    [~, row_idx(k)] = min(abs(y_vec - path_y_km(k)));
end

col_idx = max(1, min(size(Z, 2), col_idx));
row_idx = max(1, min(size(Z, 1), row_idx));

path_z_km = Z(sub2ind(size(Z), row_idx, col_idx));   % elevation at each node

%% TIME VECTOR
% key parameter n: mission start Earth day
n = 2835;
T_total = 24 * 14;
dt      = 0.2;
t       = 0:dt:T_total;
N       = length(t);

fprintf('Time vector: %d steps  (dt = %.4f hr)\n', N, dt);


%% SOLAR MODEL CONSTANTS


tau      = deg2rad(-1.545);
Y_period = 346.71;
t0       = -1.23;
L        = deg2rad(-90);
T_lunar  = 708.75;
omega    = 360 / T_lunar;

t_lunar_offset = (n - 2831) * 24;

%% BUILD THE SUN MAP
%
%   For each timestep:
%     1. Compute beta and azimuth from the solar model
%     2. If sun is below horizon the whole surface is in shadow -> skip
%     3. Call ray_march_nodes() which marches from each path node toward
%        the sun and checks whether any terrain blocks the line of sight
%     4. Store result in sun_map


fprintf('Building sun_map (%d path nodes x %d time steps)...\n', n_path, N);

sun_map = false(n_path, N);

tic
for i = 1:N

    ti = t(i);

    % Solar model
    delta = tau * sin(deg2rad((360 / Y_period) * (n + ti/24 - t0)));
    H     = deg2rad(omega * (T_lunar/2 - (ti + t_lunar_offset)));
    beta  = asin(cos(L)*cos(delta)*cos(H) + sin(L)*sin(delta));

    % Sun below horizon: entire surface in shadow
    if beta <= 0
        continue   % sun_map(:,i) stays false
    end

    phi_s = atan2(cos(delta)*sin(H), ...
                  sin(L)*cos(delta)*cos(H) - cos(L)*sin(delta));

    % Sun direction unit vector (east, north)
    sx = cos(beta) * cos(phi_s);
    sy = cos(beta) * sin(phi_s);

    % Ray march for all path nodes simultaneously
    shadow_mask = ray_march_nodes(Z, x_vec, y_vec, dx, ...
                                  path_x_km, path_y_km, path_z_km, ...
                                  sx, sy, beta);

    sun_map(~shadow_mask, i) = true;

    % Progress every 10%
    if mod(i, round(N/10)) == 0
        fprintf('  %.0f %% done  (%.1f s elapsed)\n', 100*i/N, toc);
    end
end

fprintf('Done.  Total time: %.1f s\n', toc);

%% VISUALIZATION PLOTS

sun_fraction = mean(sun_map, 2);

dists    = sqrt(diff(path_x_km).^2 + diff(path_y_km).^2);
cum_dist = [0; cumsum(dists)];

figure;
plot(cum_dist, sun_fraction * 100, 'b.-', 'MarkerSize', 8, 'LineWidth', 1)
xlabel('Distance along path (km)')
ylabel('% time in sunlight')
title('Illumination fraction per path node')
ylim([0 105])
grid on

figure;
scatter(path_x_km, path_y_km, 30, sun_fraction * 100, 'filled')
colorbar; colormap(parula); caxis([0 100])
xlabel('East (km)'); ylabel('North (km)')
title('Path coloured by % time in sunlight')
axis equal; grid on


%% SAVE

save('sun_map.mat', 'sun_map', 'path_x_km', 'path_y_km', 't', 'dt', ...
     'n_path', 'N', '-v7.3');

fprintf('\nsun_map.mat saved.\n');
fprintf('Matrix size: %d path nodes  x  %d time steps\n', n_path, N);
fprintf('Memory footprint (logical): %.1f MB\n', n_path*N / 8 / 1e6);



%% RAY MARCH FUNCTION

function shadow = ray_march_nodes(Z, x_vec, y_vec, dx, px, py, pz, sx, sy, beta_rad)
    % Inputs:
    %   Z        : curvature-corrected elevation grid [ny x nx] (km)
    %   x_vec    : 1-D east  coordinates of DEM (km)
    %   y_vec    : 1-D north coordinates of DEM (km)
    %   dx       : base DEM grid spacing (km)
    %   px, py   : east/north coordinates of query points [n_path x 1] (km)
    %   pz       : elevation of query points [n_path x 1] (km)
    %   sx, sy   : sun direction unit vector (east, north components)
    %   beta_rad : solar elevation angle (rad)
    %
    % Returns:
    %   shadow   : logic [n_path x 1], true = in shadow

    n_pts    = length(px);
    shadow   = false(n_pts, 1);
    tan_beta = tan(beta_rad);

    % Maximum ray length
    max_km = min(5.0 / tan_beta, 48.0);

    % Progressive step schedule
    %   fine  (dx steps)   for 0-5 km   : sharp shadow edges near ridges
    %   mid   (3*dx steps) for 5-20 km  : moderate precision
    %   far   (6*dx steps) beyond 20 km : coarse
    steps_fine = (dx    : dx    : min(5.0,  max_km))';
    steps_mid  = (5.0   : 3*dx  : min(20.0, max_km))';
    steps_far  = (20.0  : 6*dx  : max_km)';
    steps_far  = steps_far(steps_far > 20.0);
    all_steps  = unique([steps_fine; steps_mid; steps_far]);
    all_steps  = all_steps(all_steps > 0 & all_steps <= max_km);

    if isempty(all_steps)
        return
    end

    % active = nodes not yet confirmed shadowed (early-exit mask)
    active = true(n_pts, 1);

    for k = 1:length(all_steps)
        d_km = all_steps(k);

        if ~any(active)
            break
        end

        % position of potential blocker for all active nodes
        bx = px(active) + d_km * sx;
        by = py(active) + d_km * sy;

        % check for out of bounds (Gill test 2)
        in_bounds = bx >= x_vec(1) & bx <= x_vec(end) & ...
                    by >= y_vec(1) & by <= y_vec(end);

        % If no active node's ray is still inside the DEM, stop
        if ~any(in_bounds)
            break
        end

        % Interpolate terrain elevation at blocker position
        bz = nan(sum(active), 1);
        bz(in_bounds) = interp2(x_vec, y_vec, Z, ...
            bx(in_bounds), by(in_bounds), 'linear', NaN);

        % Main shadow test (Gill test 1)
        % Line of sight elevation at this distance
        los = pz(active) + d_km * tan_beta;
        % Shadow test: blocker above line of sight
        newly_shadowed_local = in_bounds & ~isnan(bz) & (bz > los);

        % Map back to full node indices and update
        active_idx = find(active);
        shadow(active_idx(newly_shadowed_local)) = true;
        active(active_idx(newly_shadowed_local)) = false;
    end
end
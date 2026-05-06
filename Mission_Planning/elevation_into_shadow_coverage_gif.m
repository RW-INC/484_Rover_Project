% =====================================================
% LUNAR SOUTH POLE SHADOW COVERAGE MODEL TO GIF
% =====================================================
%
% SHADOW METHOD: ray marching
%   A point P is in shadow if any terrain between P and the sun
%   rises above the line of sight: zb > zp + d * tan(beta)
%
% OPTs:
%   1. Fully vectorised — all pixels processed simultaneously
%      per ray step
%   2. Early-exit mask — once a pixel is confirmed shadowed it is
%      excluded from all future checks
%   3. Progressive step size — finer steps near the query point,
%      less fine steps further away
%   4. Capped max range — 50 km cap

clear; clc; close all;

%% LOAD TERRAIN 
load('south_pole_elevation.mat')

dx = X(1,2) - X(1,1);
fprintf('Grid spacing: %.3f km\n', dx);

x_vec = X(1,:);
y_vec = Y(:,1);


%% CURVATURE CORRECTION

R_moon = 1737.4;
Z = elevation - (X.^2 + Y.^2) / (2*R_moon);

%% SOLAR MODEL PARAMETERS

n        = 2832;
L        = deg2rad(-90);
t0       = -1.23;
Y_period = 346.71;
tau      = deg2rad(-1.545);
T_lunar  = 708.75;
T_mission = T_lunar;
dt       = 15/60;
omega    = 360 / T_lunar;


%% ROVER MOTION PARAMETERS

x_rover = -5;
y_rover = 20;
vx      = -1/35;
vy      = 0;

rover_path_x = [];
rover_path_y = [];

%% ANIMATION LOOP

window_size = 10;

for t_start = 0:window_size:(T_mission - window_size)

    t_end           = t_start + window_size;
    t               = t_start:dt:t_end;
    window_duration = t_end - t_start;

    fprintf('Simulating hours %.2f to %.2f\n', t_start, t_end);

    shadow_time = zeros(size(Z));
    tic

    for k = 1:length(t)

        ti = t(k);

        delta = tau * sin(deg2rad((360/Y_period)*(n + ti/24 - t0)));
        t_lunar_offset = (n - 2831) * 24;
        H     = deg2rad(omega * (T_lunar/2 - (ti + t_lunar_offset)));
        beta  = asin(cos(L)*cos(delta)*cos(H) + sin(L)*sin(delta));

        if beta <= 0
            shadow_time = shadow_time + dt;
            continue
        end

        phi_s = atan2(cos(delta)*sin(H), sin(L)*cos(delta)*cos(H) - cos(L)*sin(delta));

        sx = cos(beta) * cos(phi_s);
        sy = cos(beta) * sin(phi_s);

        shadow_mask = ray_march_vectorised(Z, x_vec, y_vec, dx, sx, sy, beta);

        shadow_time = shadow_time + shadow_mask * dt;

    end

    fprintf('  Window done in %.1f s\n', toc);

    %% -Sunlight percent
    shadow_percent = 100 * shadow_time / window_duration;
    sun_percent    = 100 - shadow_percent;

    %% -Plot
    fig = figure('visible','off');
    imagesc(X(1,:), Y(:,1), sun_percent);
    axis equal; axis xy
    colorbar; colormap(parula); caxis([0 100])
    xlabel('Kilometers East'); ylabel('Kilometers North')
    title(sprintf('Sunlight %.0f to %.0f hrs', t_start, t_end))

    filename = sprintf('Sunlight_%03d_to_%03d.jpg', round(t_start), round(t_end));
    exportgraphics(fig, filename, 'Resolution', 300);
    close(fig)

end

%% RAY MARCH FUNCTION

function shadow = ray_march_vectorised(Z, x_vec, y_vec, dx, sx, sy, beta_rad)
    % Inputs:
    %   Z        : curvature-corrected elevation grid [ny x nx] (km)
    %   x_vec    : 1-D east  coordinates [1 x nx] (km)
    %   y_vec    : 1-D north coordinates [ny x 1] (km)
    %   dx       : base grid spacing (km)
    %   sx, sy   : sun direction unit vector (east, north components)
    %   beta_rad : solar elevation angle in rad)
    %
    % Returns:
    %   shadow   : logic [ny x nx], true = in shadow

    [ny, nx] = size(Z);
    shadow   = false(ny, nx);
    tan_beta = tan(beta_rad);

    % Cap max ray length 
    % tried beta=1.5 deg a 5 km peak casts = 191 km,
    % but DEM is only 50 km, so cap at 48 km
    max_km   = min(5.0 / tan_beta, 48.0);

    % Progressive step schedule:
    %   - Fine steps (1 pixel = dx km) for the first 5 km
    %   - Coarser steps (3 pixels) for 5-20 km
    %   - Even coarser (6 pixels) beyond 20 km
    %  preserves sharp shadow edges near ridges while being fast
    % for long shadows from far away peaks.
    steps_fine   = (dx    : dx    : min(5.0,  max_km))';
    steps_mid    = (5.0   : 3*dx  : min(20.0, max_km))';
    steps_far    = (20.0  : 6*dx  : max_km)';
    steps_far    = steps_far(steps_far > 20.0);
    all_steps    = unique([steps_fine; steps_mid; steps_far]);
    all_steps    = all_steps(all_steps > 0 & all_steps <= max_km);

    if isempty(all_steps)
        return
    end

    % Flatten grid coords for vectorised lookup
    [X_grid, Y_grid] = meshgrid(x_vec, y_vec);
    x_flat = X_grid(:);          % (ny*nx) x 1
    y_flat = Y_grid(:);
    z_flat = Z(:);

    % Pixels not yet determined shadowed
    active = true(ny*nx, 1);

    for k = 1:length(all_steps)
        d_km = all_steps(k);

        if ~any(active)
            break
        end

        % position of potential blocker for all active nodes
        bx = x_flat(active) + d_km * sx;
        by = y_flat(active) + d_km * sy;

        % check for out of bounds (Gill test 2)
        in_bounds = bx >= x_vec(1) & bx <= x_vec(end) & ...
                    by >= y_vec(1) & by <= y_vec(end);

        % If nothing is in bounds at this range, all remaining rays have
        % left the DEM — stop entirely
        if ~any(in_bounds)
            break
        end

        % Interpolate blocker elevation (only for in-bounds pixels)
        bz = nan(sum(active), 1);
        bz(in_bounds) = interp2(x_vec, y_vec, Z, ...
            bx(in_bounds), by(in_bounds), 'linear', NaN);

        % Main shadow test (Gill test 1)
        % Line of sight elevation at this distance
        los = z_flat(active) + d_km * tan_beta;
        % Newly shadowed: blocker above line of sight
        newly_shadowed_local = in_bounds & ~isnan(bz) & (bz > los);

        % Map back to full grid indices
        active_idx = find(active);
        shadow(active_idx(newly_shadowed_local)) = true;

        % Remove shadowed pixels from active
        active(active_idx(newly_shadowed_local)) = false;
    end
end

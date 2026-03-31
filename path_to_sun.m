% ==========================
% LOAD TERRAIN
% ==========================

clear; clc; close all;

load('south_pole_elevation.mat')

size(X)
size(Y)
size(elevation)

dx = X(1,2) - X(1,1);   % grid spacing
fprintf("Grid spacing: %.3f km\n", dx);

% ==========================
% CURVATURE CORRECTION
% ==========================

R_moon = 1737.4; % km
Z = elevation - (X.^2 + Y.^2)/(2*R_moon);

% ==========================
% PRECOMPUTE TERRAIN GRADIENT
% ==========================

[dzdx, dzdy] = gradient(Z, dx);

%% ==========================
% LOAD PATH DATA
% ==========================
path_data = readmatrix('planned_path_nodes.csv');

x_path = path_data(:,1);
y_path = path_data(:,2);
num_points = length(x_path);

%% ==========================
% ROVER MOTION PARAMETERS
% ==========================
speed_mps   = 0.02;                 % m/s
speed_kmhr  = speed_mps * 3.6;      % km/hr

% Distance along path
distance = [0; cumsum(sqrt(diff(x_path).^2 + diff(y_path).^2))]*10;

% Time at each waypoint (hours)
t_path = distance / speed_mps / 3600; % convert sec → hr

% Initialize sunlight flag
sun_flag = zeros(num_points,1);

%% ==========================
% SOLAR MODEL PARAMETERS
% ==========================
n         = 2832;                   % Reference day (Oct 1, 2027)
L         = deg2rad(-90);           % South Pole latitude
t0        = -1.23;
Y_period  = 346.71;
tau       = deg2rad(-1.545);

T_lunar   = 708.75;                 % Lunar day (hours)
omega     = 360 / T_lunar;          % Deg/hr

%% ==========================
% MAIN LOOP: SUNLIGHT CHECK
% ==========================
for i = 1:num_points

    ti = t_path(i);

    % --- Solar declination ---
    delta = tau * sin(deg2rad((360/Y_period) * (n + ti/24 - t0)));

    % --- Hour angle ---
    t_lunar_offset = (n - 2831) * 24;
    H = deg2rad(omega * (T_lunar/2 - (ti + t_lunar_offset)));

    % --- Solar elevation angle ---
    beta = asin( cos(L)*cos(delta)*cos(H) + sin(L)*sin(delta) );

    % If sun is below horizon → shadow
    if beta <= 0
        sun_flag(i) = 0;
        continue
    end

    % --- Solar azimuth ---
    phi_s = atan2( ...
        cos(delta)*sin(H), ...
        sin(L)*cos(delta)*cos(H) - cos(L)*sin(delta) ...
    );

    % --- Find nearest terrain grid index ---
    [~, ix] = min(abs(X(1,:) - x_path(i)));
    [~, iy] = min(abs(Y(:,1) - y_path(i)));

    % --- Terrain slope projection in sun direction ---
    sx = cos(beta)*cos(phi_s);
    sy = cos(beta)*sin(phi_s);

    slope = dzdx(iy,ix)*sx + dzdy(iy,ix)*sy;

    % --- Shadow test ---
    if slope > tan(beta)
        sun_flag(i) = 0;
    else
        sun_flag(i) = 1;
    end

end

%% ==========================
% SAVE OUTPUT
% ==========================
output = [t_path, x_path, y_path, sun_flag];
writematrix(output, 'sun_profile.csv');

%% ==========================
% PLOT RESULTS
% ==========================
figure;
plot(t_path, sun_flag, '-o', 'LineWidth', 1.5)
xlabel('Time (hr)')
ylabel('Sunlight (1 = sun, 0 = shadow)')
title('Sunlight Along Rover Path')
grid on
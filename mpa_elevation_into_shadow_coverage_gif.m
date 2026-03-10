% =====================================================
% OPTIMIZED LUNAR SOUTH POLE SHADOW COVERAGE MODEL
% =====================================================

% ==========================
% LOAD TERRAIN
% ==========================

clear; clc; close all;

load('south_pole_elevation.mat')

size(X)
size(Y)
size(elevation)

dx = X(1,2) - X(1,1);   % grid spacing (should be 1 km now)
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

% ==========================
% SOLAR MODEL PARAMETERS
% ==========================

n = 2831;                    % Oct 1, 2027
L = deg2rad(-90);            % South Pole
t0 = -1.23;
Y_period = 346.71;
tau = deg2rad(-1.545);

T_lunar = 708.75;              % hours
T_mission = 708.75/2;              % hours
dt = 15/60;                    % 15 minutes

omega   = 360/T_lunar;


%% ==========================
% ROVER MOTION PARAMETERS
%% ==========================

x_start = -5; % km
y_start = 20; % km

vx = -1/35;  % km/hr (east-west speed)
vy = 0;  % km/hr (north-south speed)

x_rover = x_start;
y_rover = y_start;

rover_path_x = [];
rover_path_y = [];

%% ==========================
% ANIMATION LOOP
%% ==========================

window_size = 10; % hours

for t_start = 0:window_size:(T_mission - window_size)

    t_end = t_start + window_size;
    t = t_start:dt:t_end;

    window_duration = t_end - t_start;

    fprintf("Simulating hours %.2f to %.2f\n", t_start, t_end);

    shadow_time = zeros(size(Z));

    %% --- MAIN TIME LOOP ---
    for k = 1:length(t)

        ti = t(k);

        delta = tau * sin(deg2rad((360/Y_period)*(n + ti/24 - t0)));

        H = deg2rad(omega * (T_mission/2 - ti));

        beta = asin( cos(L)*cos(delta)*cos(H) + sin(L)*sin(delta) );

        if beta <= 0
            shadow_time = shadow_time + dt;
            continue
        end

        phi_s = atan2( ...
            cos(delta)*sin(H), ...
            sin(L)*cos(delta)*cos(H) - cos(L)*sin(delta) ...
        );

        sx = cos(beta)*cos(phi_s);
        sy = cos(beta)*sin(phi_s);

        terrain_slope = dzdx*sx + dzdy*sy;

        shadow_mask = terrain_slope > tan(beta);

        shadow_time = shadow_time + shadow_mask * dt;

    end

    %% --- Compute sunlight percent ---
    shadow_percent = 100 * shadow_time / window_duration;
    sun_percent = 100 - shadow_percent;

    %% ==========================
    % UPDATE ROVER POSITION
    %% ==========================

    x_rover = x_rover + vx*window_size;
    y_rover = y_rover + vy*window_size;

    rover_path_x = [rover_path_x x_rover];
    rover_path_y = [rover_path_y y_rover];

    %% ==========================
    % PLOT
    %% ==========================

    fig = figure('visible','off');

    imagesc(X(1,:), Y(:,1), sun_percent);
    axis equal
    axis xy
    colorbar
    colormap(parula)
    caxis([0 100])

    xlabel('Kilometers East')
    ylabel('Kilometers North')

    title(sprintf('Sunlight %.0f to %.0f hrs', t_start, t_end))

    hold on

    % Rover path
    plot(rover_path_x, rover_path_y, 'w-', 'LineWidth',2)

    % Rover current location
    plot(x_rover, y_rover, 'ro', ...
        'MarkerSize',10, ...
        'MarkerFaceColor','r')

    hold off

    %% --- Save JPEG ---
    filename = sprintf('Sunlight_%03d_to_%03d.jpg', ...
        round(t_start), round(t_end));

    exportgraphics(fig, filename, 'Resolution', 300);
    close(fig)

end
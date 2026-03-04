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

n = 2831;                     % Oct 1, 2027
L = deg2rad(-90);            % South Pole
t0 = -1.23;
Y_period = 346.71;
tau = deg2rad(-1.545);

T_lunar = 708.75/2;              % hours
dt = 15/60;                    % 15 minutes

% ==========================
% TIME WINDOW CONTROL
% ==========================

% Fractional window options:
% 0–1 scale across lunar day

%window_fraction_start = 0;   % 0 = start of day, 0.5 = halfway
%window_fraction_end   = 0.5;    % 1 = end of day

%t_start = window_fraction_start * T_lunar;
%t_end   = window_fraction_end   * T_lunar;
i = 0;

for i in T_lunar
    % exact window option
    t_start = i; % type hour to start window, 0 = start of day
    t_end   = i + 10; % type hour to end window, 354 = end of mission
    i = i + 10;

    t = t_start:dt:t_end;
    window_duration = t_end - t_start;

    fprintf("Simulating hours %.2f to %.2f\n", t_start, t_end);

    N = length(t);

    shadow_time = zeros(size(Z));

%% ==========================
% MAIN TIME LOOP
% ==========================

for k = 1:N
    
    ti = t(k);

    % Declination
    delta = tau * sin( deg2rad((360/Y_period)*(n + ti/24 - t0)) );

    % Hour angle
    H = deg2rad(0.515 * (ti - T_lunar/2));

    % Solar elevation
    beta = asin( cos(L)*cos(delta)*cos(H) + sin(L)*sin(delta) );

    if beta <= 0
        shadow_time = shadow_time + dt;
        continue
    end

    % Solar azimuth
    cosB = cos(beta);
    val = cos(delta)*sin(H)/cosB;
    val = max(-1,min(1,val));
    phi_s = asin(val);

    % Sun horizontal direction
    sx = cos(phi_s);
    sy = sin(phi_s);

    % Terrain slope toward sun
    terrain_slope = dzdx*sx + dzdy*sy;

    % Shadow condition
    shadow_mask = terrain_slope > tan(beta);

    shadow_time = shadow_time + shadow_mask * dt;

end

%% ==========================
% SHADOW PERCENTAGE
% ==========================

shadow_percent = 100 * shadow_time / window_duration;

%% ==========================
% 2D VISUALIZATION
% ==========================

%figure;
%imagesc(X(1,:), Y(:,1), shadow_percent);
%axis equal
%axis xy
%colorbar
%colormap(parula)
%xlabel('Kilometers East')
%ylabel('Kilometers North')
%title('Percent of Lunar Day in Shadow')

sun_percent = 100 - shadow_percent;

figure;
imagesc(X(1,:), Y(:,1), sun_percent);
axis equal
axis xy
colorbar
colormap(parula)
xlabel('Kilometers East')
ylabel('Kilometers North')
title(sprintf('Percent Sunlight Hours %.2f to %.2f', t_start, t_end))

%% ==========================
% 3D VISUALIZATION (OPTIONAL)
% ==========================

%figure;
%surf(X,Y,Z,shadow_percent,'EdgeColor','none')
%axis equal
%view(45,30)
%colorbar
%title('3D Shadow Coverage')

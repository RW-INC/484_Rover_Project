clear; clc; close all;

%% USER PARAMETERS

battery_mass = 1.66;               % kg 
batt_specific_energy = 248.6;      % Wh/kg
E_batt_capacity = battery_mass * batt_specific_energy;

initial_SOC_fraction = 0.90;     % 90% charged at landing

speed = 0.02;                    % m/s
P_motors = 25;                   % W
P_av_travel = 19;                % W
P_av_rest = 1.5;                 % W
P_heaters = 0.2;                 % W
payload_total_power = 15;        % W
num_of_payloads = 2;

payload_drop_distances = [10000 10000];
charge_stop_fraction  = 0.80;     % Stop charging at 80%

total_distance_goal = 10000;       % meters

%% PANEL PROPERTIES

panel_number  = 3;
panel_widths  = [0.25, 0.3, 0.3];
panel_height  = 0.31;

lidar_cutout = 0.06*0.06; % 60mm by 60mm square

panel_eff   = 0.32;
inc_derate  = 0.95;

%% TIME + SOLAR CONSTANTS

n       = 2831; % Oct. 1st 2027
Tau     = deg2rad(-1.545);
Y       = 346.71;
t_0     = -1.23;

T_total = 24*14; % 1 lunar day
dt      = 0.01;                  % hours (~36 sec)
t       = 0:dt:T_total;
N       = length(t);

omega   = 360 / T_total;

%% MOON GEOMETRY

R_moon = 1737400;
L      = deg2rad(-91);
E      = deg2rad(90);

%% SOLAR CONSTANTS

G_sc    = 1361;
I_illum = 1;

load('south_pole_elevation.mat')

dx_grid = X(1,2) - X(1,1);

R_moon = 1737.4; % km
Z = elevation - (X.^2 + Y.^2)/(2*R_moon);

azi_p = linspace(0,360,panel_number+1);
azi_p(end) = [];
azi_p = deg2rad(azi_p);

theta_dip = acos(R_moon/(R_moon + panel_height));

%% LOAD PATH + SUN DATA


data = readmatrix('sun_profile.csv');
t_path = data(:,1);     % hours
x_path = data(:,2);
y_path = data(:,3);
sun_flag = zeros(N,1);

N = length(t_path);

dt = mean(diff(t_path));   % time step from path

for i = 1:N

    % Find nearest grid index
    [~, ix] = min(abs(X(1,:) - x_path(i)));
    [~, iy] = min(abs(Y(:,1) - y_path(i)));

    % Store indices
    path_ix(i) = ix;
    path_iy(i) = iy;

end


%% SOLAR POWER (PATH-BASED)

P_solar = zeros(1,N);

for i = 1:N

    ti = t_path(i);

    % Solar angles
    delta = Tau * sin(deg2rad(360/Y * (n + ti/24 - t_0)));

    t_lunar_offset = (n - 2831) * 24;

    H = deg2rad(omega * (T_total/2 - (ti + t_lunar_offset)));

    beta = asin(cos(L)*cos(delta)*cos(H) + sin(L)*sin(delta));

    if beta <= 0
        sun_flag(i) = 0;
        continue
    end

    phi_s = atan2( ...
        cos(delta)*sin(H), ...
        sin(L)*cos(delta)*cos(H) - cos(L)*sin(delta) ...
    );

    % --- RAY TRACE HERE ---
    sun_flag(i) = isPointInSun_raytrace( ...
        Z, dx_grid, beta, phi_s, ...
        path_iy(i), path_ix(i) ...
    );

end

    P_total = 0;

    if B > -theta_dip

        for p = 1:panel_number

            cos_theta = cos(B)*cos(azi_s - azi_p(p)) + sin(B)*cos(E);

            if cos_theta > 0

                G_panel = G_sc * cos_theta * I_illum;

                panel_area = panel_widths(p) * panel_height - lidar_cutout;

                P_panel = G_panel ...
                        * panel_area ...
                        * panel_eff ...
                        * inc_derate;

                P_total = P_total + P_panel;

            end
        end
    end

    P_solar(i) = P_total;



%% SOC SIMULATION WITH PATH + SMART CHARGING

SOC = zeros(1,N);
SOC(1) = initial_SOC_fraction * E_batt_capacity;

distance = zeros(1,N);

payloads_remaining = num_of_payloads;
next_payload_index = 1;

in_charging = false;
waiting_for_sun = false;

for i = 2:N

    current_distance = distance(i-1);

    % -------------------------------
    % PAYLOAD DROP
    % -------------------------------
    if next_payload_index <= length(payload_drop_distances)
        if current_distance >= payload_drop_distances(next_payload_index)
            payloads_remaining = payloads_remaining - 1;
            next_payload_index = next_payload_index + 1;
        end
    end

    % -------------------------------
    % SUN CONDITION
    % -------------------------------
    in_sun = (sun_flag(i) == 1);

    % -------------------------------
    % SMART CHARGING LOGIC
    % -------------------------------

    % --- Trigger logic ---
    if ~in_charging

        if SOC(i-1) <= 0.30 * E_batt_capacity

            if in_sun
                in_charging = true;

            else
                waiting_for_sun = true;
            end
        end
    end

    % --- Waiting for sun ---
    if waiting_for_sun

        if in_sun || SOC(i-1) <= 0.10 * E_batt_capacity
            in_charging = true;
            waiting_for_sun = false;
        end
    end

    % --- Stop charging ---
    if in_charging && SOC(i-1) >= charge_stop_fraction * E_batt_capacity
        in_charging = false;
    end

    % -------------------------------
    % OPERATING MODE
    % -------------------------------

    if in_charging
        P_mode = P_av_rest;
        driving_now = false;

    else
        P_mode = P_motors + P_av_travel;
        driving_now = true;
    end

    % -------------------------------
    % PAYLOAD POWER
    % -------------------------------
    P_payload = payloads_remaining * (payload_total_power/num_of_payloads);

    % -------------------------------
    % TOTAL LOAD
    % -------------------------------
    P_load = P_mode + P_payload + P_heaters;

    % -------------------------------
    % DISTANCE UPDATE (FOLLOW PATH)
    % -------------------------------
    if driving_now && i > 1
        dx = (x_path(i) - x_path(i-1)) * 1000;
        dy = (y_path(i) - y_path(i-1)) * 1000;
        distance(i) = distance(i-1) + sqrt(dx^2 + dy^2);
    else
        distance(i) = distance(i-1);
    end

    % -------------------------------
    % SOC UPDATE
    % -------------------------------
    P_net = P_solar(i-1) - P_load;

    SOC(i) = SOC(i-1) + P_net * dt;

    % Limits
    SOC(i) = min(E_batt_capacity, max(0, SOC(i)));

end

%% RESULTS

min_SOC = min(SOC);
max_SOC = max(SOC);

fprintf('Minimum SOC (Wh): %.2f\n', min_SOC);
fprintf('Maximum SOC (Wh): %.2f\n', max_SOC);

if min_SOC <= 0
    fprintf('⚠ Battery depleted during mission.\n');
else
    fprintf('Mission feasible with this battery mass.\n');
end

if current_distance < total_distance_goal
    fprintf('⚠ Distance goal not reached.\n');
else
    fprintf('Distance goal reached.\n');
end

%% PLOTS

figure;
plot(t,SOC,'LineWidth',2)
xlabel('Time [hr]')
ylabel('Battery Energy (Wh)')
title('Battery SOC vs Time')
grid on

figure;
plot(t,distance,'LineWidth',2)
xlabel('Time [hr]')
ylabel('Distance Traveled [m]')
title('Distance vs Time')
grid on


figure;
subplot(3,1,1)
plot(t_path, SOC)
ylabel('SOC (Wh)')
grid on

subplot(3,1,2)
plot(t_path, sun_flag)
ylabel('Sun (1/0)')
grid on

subplot(3,1,3)
plot(t_path, distance)
ylabel('Distance (m)')
xlabel('Time (hr)')
grid on

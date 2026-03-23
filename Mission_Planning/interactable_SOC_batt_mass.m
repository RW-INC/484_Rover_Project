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
nav_interval = 10;                % meters
nav_duration = 5/60;              % hours (5 minutes)
charge_start_fraction = 0.10;     % Start charging at 10%
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

%% SOLAR POWER MODEL

azi_p = linspace(0,360,panel_number+1);
azi_p(end) = [];
azi_p = deg2rad(azi_p);

theta_dip = acos(R_moon/(R_moon + panel_height));
P_solar = zeros(1,N);

for i = 1:N

    ti = t(i);

    delta = Tau * sin(deg2rad(360/Y * (n + ti/24 - t_0)));

    t_lunar_offset = (n - 2831) * 24;   % Earth days -> hours

    H = deg2rad(omega * (T_total/2 - (ti + t_lunar_offset)));

    B = asin(cos(L)*cos(delta)*cos(H) + sin(L)*sin(delta));
    cosB = cos(B);

    if abs(cosB) > 1e-10
        azi_s = asin(max(-1, min(1, cos(delta)*sin(H)/cosB)));
    else
        azi_s = 0;
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

end

%% MISSION STATE VARIABLES

SOC = zeros(1,N);
SOC(1) = initial_SOC_fraction * E_batt_capacity;

distance = zeros(1,N);

payloads_remaining = num_of_payloads;
next_payload_index = 1;

next_nav_distance = nav_interval;
in_navigation = false;
nav_timer = 0;

in_charging = false;


%% MAIN TIME LOOP

for i = 2:N

    current_distance = distance(i-1);

    %---------------------------------
    % CHECK IF MISSION COMPLETE
    % ---------------------------------
    if current_distance >= total_distance_goal
        driving = false;
    else
        driving = true;
    end

    % ---------------------------------
    % PAYLOAD DROP LOGIC
    % ---------------------------------
    if next_payload_index <= length(payload_drop_distances)
        if current_distance >= payload_drop_distances(next_payload_index)
            payloads_remaining = payloads_remaining - 1;
            next_payload_index = next_payload_index + 1;
        end
    end

    % ---------------------------------
    % NAVIGATION TRIGGER
    % ---------------------------------
    if ~in_navigation && ~in_charging && driving && ...
            current_distance >= next_nav_distance

        in_navigation = true;
        nav_timer = nav_duration;
        next_nav_distance = next_nav_distance + nav_interval;
    end

    % ---------------------------------
    % NAVIGATION COUNTDOWN
    % ---------------------------------
    if in_navigation
        nav_timer = nav_timer - dt;
        if nav_timer <= 0
            in_navigation = false;
        end
    end

    % ---------------------------------
    % SOC-BASED CHARGING LOGIC
    % ---------------------------------
    
    % Trigger charging if SOC drops below threshold
    if ~in_navigation && ~in_charging && driving && ...
            SOC(i-1) <= charge_start_fraction * E_batt_capacity
    
        in_charging = true;
    end
    
    % Stop charging when sufficiently recharged
    if in_charging && ...
            SOC(i-1) >= charge_stop_fraction * E_batt_capacity
    
        in_charging = false;
    end


    % ---------------------------------
    % DETERMINE OPERATING MODE
    % Priority: NAV > CHARGE > DRIVE > IDLE
    % ---------------------------------

    if in_navigation
        P_mode = P_motors + P_av_travel;
        driving_now = false;

    elseif in_charging
        P_mode = P_av_rest;     % only rest avionics while charging
        driving_now = false;

    elseif driving
        P_mode = P_motors + P_av_travel;
        driving_now = true;

    else
        P_mode = P_av_rest;     % mission complete idle
        driving_now = false;
    end

    % ---------------------------------
    % PAYLOAD POWER (Dynamic)
    % ---------------------------------
    P_payload = payloads_remaining * (payload_total_power/num_of_payloads);

    % ---------------------------------
    % TOTAL LOAD POWER
    % ---------------------------------
    P_load = P_mode + P_payload + P_heaters;

    % ---------------------------------
    % UPDATE DISTANCE
    % ---------------------------------
    if driving_now
        distance(i) = current_distance + speed * dt * 3600;
    else
        distance(i) = current_distance;
    end

    % ---------------------------------
    % SOC UPDATE
    % ---------------------------------
    P_net = P_solar(i-1) - P_load;
    SOC(i) = SOC(i-1) + P_net * dt;

    % ---------------------------------
    % BATTERY LIMITS
    % ---------------------------------
    if SOC(i) > E_batt_capacity
        SOC(i) = E_batt_capacity;
    end

    if SOC(i) < 0
        SOC(i) = 0;
    end

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

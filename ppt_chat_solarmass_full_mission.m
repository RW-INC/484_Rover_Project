clear; clc; close all;

%% TIME AND SOLAR CONSTANTS

n       = 2831;                  % October 1, 2027 reference day
Tau     = deg2rad(-1.545);
Y       = 346.71;
t_0     = -1.23;

T_total = 708.75;               % lunar day [hr]
dt      = 0.25;
t       = 0:dt:T_total;
N       = length(t);

omega   = 360 / T_total;        % deg/hr

%% MOON GEOMETRY

R_moon    = 1737400;            % meters
h_panel   = 0.3;                % meters
theta_dip = acos(R_moon/(R_moon + h_panel));  % radians

L = deg2rad(-91);               % latitude (slightly beyond pole for numerical stability)
E = deg2rad(90);                % vertical panels

%% SOLAR CONSTANTS

G_sc    = 1361;                 % W/m^2
I_illum = 0.3543;               % illumination factor

%% PANEL PROPERTIES

panel_number = 4;

panel_width  = 0.35;            % meters
panel_height = 0.6;             % meters

panel_area_single = panel_width * panel_height;

panel_eff   = 0.28;
inc_derate  = 0.95;

azi_p = deg2rad([0, 90, 180, 270]);      % panel azimuth orientations

%% BATTERY PROPERTIES

panel_mass_per_m2     = 2.0;    % kg/m^2
batt_specific_energy  = 200;    % Wh/kg

%% PREALLOCATE ARRAYS

B_arr      = zeros(1,N);
G_flux_arr = zeros(1,N);
P_elec_arr = zeros(1,N);

%% MAIN LOOP

for i = 1:N

    ti = t(i);

    % Solar declination
    delta = Tau * sin(deg2rad(360/Y * (n + ti/24 - t_0)));

    % Hour angle
    H = deg2rad(omega * (T_total/2 - ti));

    % Solar elevation
    B = asin(cos(L)*cos(delta)*cos(H) + sin(L)*sin(delta));
    B_arr(i) = B;

    % Solar azimuth
    cosB = cos(B);

    if abs(cosB) > 1e-10
        azi_s = asin(max(-1, min(1, cos(delta)*sin(H)/cosB)));
    else
        azi_s = 0;
    end

    %% Compute total electrical power from all panels

    P_total = 0;
    G_total = 0;

    % Check if Sun is above raised horizon
    if B > -theta_dip

        for p = 1:panel_number

            cos_theta = cos(B)*cos(azi_s - azi_p(p)) + sin(B)*cos(E);

            if cos_theta > 0

                % Irradiance contribution from this panel
                G_panel = G_sc * cos_theta * I_illum;

                % Electrical power from this panel
                P_panel = G_panel ...
                          * panel_area_single ...
                          * panel_eff ...
                          * inc_derate;

                P_total = P_total + P_panel;
                G_total = G_total + G_panel;

            end

        end

    end

    G_flux_arr(i) = G_total;
    P_elec_arr(i) = P_total;

end

%% POWER STATISTICS

P_avg = mean(P_elec_arr);
P_max = max(P_elec_arr);

disp(['Average electrical power: ', num2str(P_avg), ' W'])
disp(['Max electrical power: ', num2str(P_max), ' W'])

%% ENERGY OVER FULL LUNAR DAY

t_mission = 24*14; % full 14 day mission time

E_solar = P_avg * t_mission;

disp(['Total solar energy generated: ', num2str(E_solar), ' Wh'])

%% ENERGY BUDGET

speeds = [0.005, 0.010, 0.015, 0.020, 0.025];   % m/s

P_base = 25.0; % W

t_trav = (1000 ./ speeds) / 3600;

E_base = P_base .* t_trav;

E_avionics_travel = 10*t_trav;
E_avionics_rest = 1.5*(t_mission-t_trav);
E_heaters = 0.2*t_mission;
E_payloads = 4*15*t_mission;
E_motors = min(E_base);

E_total = E_avionics_travel + E_avionics_rest + E_heaters + E_payloads + E_motors

E_required = E_total - E_solar;

disp(['Total mission energy required: ', num2str(E_total), ' Wh'])
disp(['Energy deficit (battery must supply): ', num2str(E_required), ' Wh'])

%% BATTERY MASS REQUIRED
mass_batt_no_panels = E_total / batt_specific_energy;
mass_batt_panels = E_required / batt_specific_energy;

disp(['Battery mass required w/ no panels: ', num2str(mass_batt_no_panels), ' kg'])
disp(['Battery mass requiredw/ panels: ', num2str(mass_batt_panels), ' kg'])

%% PANEL MASS

panel_mass = panel_area_single * panel_number * panel_mass_per_m2;

disp(['Solar panel mass: ', num2str(panel_mass), ' kg'])

%% PLOTS

figure;
plot(t, rad2deg(B_arr), 'LineWidth',2)
xlabel('Time [hr]')
ylabel('Solar elevation angle [deg]')
title('Solar elevation vs Time')
grid on

figure;
plot(t, G_flux_arr, 'LineWidth',2)
xlabel('Time [hr]')
ylabel('Solar flux [W/m^2]')
title('Solar flux vs Time')
ylim([0 1400])
grid on

figure;
plot(t, P_elec_arr, 'LineWidth',2)
xlabel('Time [hr]')
ylabel('Electrical Power [W]')
title('Electrical Power vs Time')
grid on

clear; clc; close all;

%% TIME AND SOLAR CONSTANTS

n       = 2831;
Tau     = deg2rad(-1.545);
Y       = 346.71;
t_0     = -1.23;

T_total = 24*14;
dt      = 0.25;
t       = 0:dt:T_total;
N       = length(t);

omega   = 360 / T_total;

%% MOON GEOMETRY

R_moon = 1737400;
L      = deg2rad(-91);
E      = deg2rad(90);

%% SOLAR CONSTANTS

G_sc    = 1361;
I_illum = 0.3543;

%% PANEL PROPERTIES

panel_number  = 3;
panel_widths  = [0.4, 0.45, 0.45];
panel_height  = 0.45 - 0.0059 - 0.05;

panel_eff   = 0.28;
inc_derate  = 0.95;

azi_p = linspace(0,360,panel_number+1);
azi_p(end) = [];
azi_p = deg2rad(azi_p);

%% BATTERY

batt_specific_energy = 269;

%% MISSION CONSTANTS

t_mission = 24*14;

speed  = 0.02;
P_base = 25.0;
d_trav = 1000;

t_trav = (d_trav / speed) / 3600;

%% NAVIGATION TIME SWEEP (hours)

t_nav = 0:1:(t_mission - t_trav);

%% COMPUTE SOLAR ENERGY ONCE

theta_dip = acos(R_moon/(R_moon + panel_height));
P_elec_arr = zeros(1,N);

for i = 1:N

    ti = t(i);

    delta = Tau * sin(deg2rad(360/Y * (n + ti/24 - t_0)));
    H     = deg2rad(omega * (T_total/2 - ti));

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

                panel_area = panel_widths(p) * panel_height;

                P_panel = G_panel ...
                        * panel_area ...
                        * panel_eff ...
                        * inc_derate;

                P_total = P_total + P_panel;

            end
        end
    end

    P_elec_arr(i) = P_total;

end

E_solar = trapz(t, P_elec_arr);

%% PREALLOCATE

mass_batt_vs_t_nav = zeros(size(t_nav));

%% LOOP OVER NAV TIME

for k = 1:length(t_nav)

    t_nav_k = t_nav(k);

    E_motors = P_base * t_trav;

    E_avionics_travel = 10 * t_trav;

    E_avionics_nav = 10 * t_nav_k;

    E_avionics_rest = 1.5 * (t_mission - t_trav - t_nav_k);

    E_heaters  = 0.2 * t_mission;

    E_payloads = 15 * t_mission;

    E_total = E_motors + ...
              E_avionics_travel + ...
              E_avionics_nav + ...
              E_avionics_rest + ...
              E_heaters + ...
              E_payloads;

    E_required = max(0, E_total - E_solar);

    mass_batt_vs_t_nav(k) = E_required / batt_specific_energy;

end

%% PLOT

figure; hold on;

plot(t_nav, mass_batt_vs_t_nav, ...
     'LineWidth',3,'Color','blue', ...
     'DisplayName','Battery Mass');

yline(2,'r--','LineWidth',3,'DisplayName','2 kg Limit');

xlabel('Navigation Time [hr]')
ylabel('Required Battery Mass [kg]')
title('Battery Mass vs Navigation Time')

grid on
legend show

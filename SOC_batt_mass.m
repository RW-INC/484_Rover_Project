clear; clc; close all;

%% TIME AND SOLAR CONSTANTS

n       = 2831;
Tau     = deg2rad(-1.545);
Y       = 346.71;
t_0     = -1.23;

T_total = 24*14;
dt      = 0.25;                 % hours
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
panel_height  = 0.35;

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
d_trav = 2000;       % fixed traversal distance

t_trav = (d_trav / speed) / 3600;   % hours

%% COMPUTE SOLAR POWER VS TIME

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

%% -------------------------
%% DEFINE LOAD POWER VS TIME
%% -------------------------

P_load_arr = zeros(1,N);

for i = 1:N

    ti = t(i);

    % Travel period
    if ti <= t_trav
        P_load_arr(i) = P_base + 10;    % motors + travel avionics
    else
        P_load_arr(i) = 1.5;            % rest avionics
    end

    % Add constant loads
    P_load_arr(i) = P_load_arr(i) + 0.2 + 15;   % heaters + payload

end

%% -------------------------
%% SOC SIMULATION
%% -------------------------

SOC = zeros(1,N);     % Wh
SOC(1) = 0;           % start reference at 0

for i = 2:N

    P_net = P_elec_arr(i-1) - P_load_arr(i-1);

    SOC(i) = SOC(i-1) + P_net * dt;   % Wh

end

%% -------------------------
%% BATTERY SIZING FROM SOC
%% -------------------------

E_batt_required = max(SOC) - min(SOC);

mass_batt = E_batt_required / batt_specific_energy;

disp(['Required Battery Mass (kg): ', num2str(mass_batt)])

%% -------------------------
%% PLOTS
%% -------------------------

figure;
plot(t, SOC,'LineWidth',2)
xlabel('Time [hr]')
ylabel('Battery Energy (Wh)')
title('Battery State of Charge vs Time')
grid on

clear; clc; close all;

n       = 2831;
Tau     = deg2rad(-1.545);
Y       = 346.71;
t_0     = -1.23;
azi_p   = deg2rad([0, 180]);
T_total = 708.75;
dt      = 0.25;
t       = 0:dt:T_total;
N       = length(t);

R_moon    = 1737400;
h_panel   = 0.3;
theta_dip = acos(R_moon / (R_moon + h_panel));

L       = deg2rad(-91);
omega   = 360 / 708.75;
E       = deg2rad(90);
G_sc    = 1361;
I_illum = 0.3543;

panel_number = 2;

panel_area     = 0.35*.6*panel_number;
panel_eff      = 0.28;
inc_derate     = 0.95;
panel_mass_per_m2 = 2.0;   % kg/m2
batt_specific_energy = 200; % Wh/kg 

B_arr      = zeros(1, N);
G_flux_arr = zeros(1, N);
P_elec_arr = zeros(1, N);

for i = 1:N
    ti = t(i);
    d  = Tau * sin(deg2rad(360/Y * (n + ti/24 - t_0)));
    H  = deg2rad(omega * (T_total/2 - ti));
    B  = asin(cos(L)*cos(d)*cos(H) + sin(L)*sin(d));
    B_arr(i) = B;
    cb = cos(B);

    if abs(cb) > 1e-10
        azi_s = asin(max(-1, min(1, cos(d)*sin(H)/cb)));
    else
        azi_s = 0;
    end

    G_total = 0;
    if B > -theta_dip
        for p = 1:2
            cos_theta = cos(B)*cos(azi_s - azi_p(p)) + sin(B)*cos(E);
            if cos_theta > 0
                G_total = G_total + G_sc * cos_theta;
            end
        end
    end

    G_flux_arr(i) = G_total * I_illum;
    P_elec_arr(i) = G_flux_arr(i) * panel_area * panel_eff * inc_derate;
end

P_avg = mean(P_elec_arr);
P_max = max(P_elec_arr);

% Energy budget per 1 km traverse
speeds    = [0.005, 0.010, 0.015, 0.020, 0.025];
P_base    = 25.0;
t_trav    = min((1000 ./ speeds) / 3600);   % hours
E_base    = P_base .* t_trav;

% total Whrs
t_mission = 24*14; % full 14 day mission time
E_avionics_travel = 10*t_trav;
E_avionics_rest = 1.5*(t_mission-t_trav);
E_heaters = 0.2*t_mission;
E_payloads = 2*15*t_mission;
E_motors = min(E_base);

E_total = E_avionics_travel + E_avionics_rest + E_heaters + E_payloads + E_motors
E_solar = P_avg*t_mission
E_required = E_total - E_solar

mass_batt_no_solar = E_total/batt_specific_energy

mass_batt_solar = E_required/batt_specific_energy


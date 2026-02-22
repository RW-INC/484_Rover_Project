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
panel_widths = [0.40, .45, .45];
panel_heights = 0.15:0.01:2;

panel_eff   = 0.28;
inc_derate  = 0.95;

% Automatically distribute panels evenly
azi_p = linspace(0, 360, panel_number+1);
azi_p(end) = [];
azi_p = deg2rad(azi_p);
%% BATTERY

batt_specific_energy = 269;

%% MISSION CONSTANTS

t_mission = 24*14;

speeds = [0.005, 0.010, 0.015, 0.020, 0.025];
P_base = 25.0;

t_trav = min((1000 ./ speeds) / 3600);
E_base = P_base .* t_trav;

E_avionics_travel = 10*t_trav
E_avionics_rest   = 1.5*(t_mission - t_trav)
E_heaters         = 0.2*t_mission
E_motors          = min(E_base)

%% PAYLOAD SWEEP

payload_counts = 1:4;

mass_batt_vs_height = zeros(length(payload_counts), length(panel_heights));

%% LOOP OVER PAYLOAD COUNT

for j = 1:length(payload_counts)

    payload_num = payload_counts(j);

    % Update total energy requirement for this payload count
    E_payloads = payload_num * 15 * t_mission

    E_total = E_avionics_travel + ...
              E_avionics_rest   + ...
              E_heaters         + ...
              E_payloads        + ...
              E_motors

    %% HEIGHT SWEEP LOOP

    for k = 1:length(panel_heights)

        panel_height = panel_heights(k);

        theta_dip = acos(R_moon/(R_moon + panel_height));

        P_elec_arr = zeros(1,N);

        %% TIME LOOP

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

        %% SOLAR ENERGY

        E_solar = trapz(t, P_elec_arr)

        %% BATTERY MASS

        E_required = max(0, E_total - E_solar)

        mass_batt_vs_height(j,k) = E_required / batt_specific_energy

    end

end

%% PLOT ALL CURVES

figure; hold on;

colors = lines(length(payload_counts));

for j = 1:length(payload_counts)

    plot(panel_heights, ...
         mass_batt_vs_height(j,:), ...
         'LineWidth',3, ...
         'Color',colors(j,:), ...
         'DisplayName',[num2str(payload_counts(j)) ' Payload(s)']);

end

% Add 2 kg reference line
yline(2, 'r--', 'LineWidth', 3, 'DisplayName', '2 kg Battery Limit');
xline(0.45, 'r--', 'LineWidth', 3, 'DisplayName', '0.45 m Height Limit');

xlabel('Solar Panel Height [m]')
ylabel('Required Battery Mass [kg]')
title('Battery Mass vs Solar Panel Height for Different Payload Counts')

legend show
grid on

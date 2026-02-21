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

panel_area     = (0.45 * 0.15) * .75;
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

% Electrical power output
figure;
plot(t, P_elec_arr, 'b', 'LineWidth', 2);
hold on;
yline(P_avg, 'r--', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
hold off;
xlabel('Time [hr]');
ylabel('Electrical Power [W]');
title('Electrical Power Output');
legend('Power output', sprintf('Average: %.2f W', P_avg), 'Location', 'best');
grid on;
ylim([0, inf]);

% Energy budget per 1 km traverse
speeds    = [0.005, 0.010, 0.015, 0.020, 0.025];
P_base    = 25;
t_trav    = (1000 ./ speeds) / 3600;   % hours
E_solar   = P_avg  .* t_trav;
E_base    = P_base .* t_trav;
E_required = max(E_base - E_solar, 0); % battery must supply this

figure;
x = 1:length(speeds);
b = bar(x, [E_solar; E_required]', 'stacked');
b(1).FaceColor = [0.20 0.63 0.17];
b(2).FaceColor = [0.84 0.15 0.16];
xticks(x);
xticklabels(arrayfun(@(s) sprintf('%.3f m/s', s), speeds, 'UniformOutput', false));
xlabel('Rover Speed');
ylabel('Energy [Wh]');
title('Energy Budget per 1 km Traverse — Region 7');
legend('Solar energy provided [Wh]', 'Battery required [Wh]', 'Location', 'northeast');
grid on;

% Mass comparison: with vs. without solar
panel_mass     = 2*(panel_area * panel_mass_per_m2);
batt_no_solar  = E_base     ./ batt_specific_energy;
batt_w_solar   = (E_required ./ batt_specific_energy) + panel_mass;

figure;
bar_data = [batt_no_solar; batt_w_solar; repmat(panel_mass, 1, length(speeds))]';
b2 = bar(x, bar_data);
b2(1).FaceColor = [0.84 0.15 0.16];
b2(2).FaceColor = [0.20 0.63 0.17];
b2(3).FaceColor = [0.12 0.47 0.71];
ylabel('Mass [kg]');
xticks(x);
xticklabels(arrayfun(@(s) sprintf('%.3f m/s', s), speeds, 'UniformOutput', false));
xlabel('Rover Speed');
title('Mass Comparison: Battery vs Solar Panel');
legend('Battery mass (no solar)', 'Battery + Solar mass (with solar)', ...
       'Solar panel mass', 'Location', 'northeast');
grid on;
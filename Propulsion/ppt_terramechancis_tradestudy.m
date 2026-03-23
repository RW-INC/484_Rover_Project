% Terramechanics Trade Studies
% This script evaluates drawbar pull and sinkage performance for a small lunar rover across a range of wheel geometries and terrain conditions.

clear; clc; close all;

% Lunar Soil Parameters 
k_c     = 1400;        % Cohesive modulus (N/m^2)
k_phi   = 820000;      % Frictional modulus (N/m^3)
n       = 1;           % Sinkage exponent
c       = 170;         % Soil cohesion (N/m^2)
gamma   = 2470;        % Soil unit weight (N/m^3)
phi     = 35*pi/180;   % Internal friction angle (rad)
K_shear = 0.018;       % Shear deformation modulus (m)

% Bearing capacity factors
K_c        = 33.37;
K_gamma   = 72.77;
N_c        = 48.09;
N_q        = 32.23;
N_gamma_t = 33.27;

% Rover Parameters
m_rover   = 8;        % Max Rover mass (kg)
m_payload = 4;        % Max Payload mass (kg)
m_tot     = m_rover + m_payload;
max_rover_length = 0.6;   % Maximum allowable rover length (m)
g = 1.62;                 % Lunar gravity (m/s^2)

% Trade Study Parameters
% Wheel geometry ranges
wheel_widths    = linspace(0.02,0.12,20);    % wheel width sweep (m)
wheel_diameters = [0.12 0.14 0.16 0.18 0.20 0.22 0.24 0.26];
wheel_cases     = [4 6];                 % total wheel count

% Terrain slopes
slope_flat  = 0; % degrees
slope_slope = 10; % degrees

% Slip ratios 
s_flat  = 0.3; % for flat slope
s_slope = 0.45; % for angled slope

% Grouser trade parameters
grouser_numbers = [8 12 16 24]; % nummber of grousers
grouser_heights = [0.005 0.01 0.015 0.02]; % grouser heights

% Drawbar Pull 
% function wrapper for clarity in parametric sweeps
compute_DP = @(b,D,N_w,theta_s,grousers,N_grouser,h_g)local_drawbar_pull(b,D,N_w,theta_s,grousers,N_grouser,h_g,m_tot,g,k_c,k_phi,n,c,gamma,phi,K_shear,K_c,K_gamma,N_c,N_q,N_gamma_t,(theta_s==0)*s_flat + (theta_s~=0)*s_slope);

% Sinkage Calculation
% Computes static sinkage based on Bekker theory
compute_z = @(b,D,N_w,theta_s)((3/(3-n))*((m_tot*g*cosd(theta_s)/N_w)/((k_c + b*k_phi)*sqrt(D)))).^(2/(2*n+1));

% 1) No Grousers – Flat & 10° Slope
for slope = [slope_flat slope_slope]
    for N_w = wheel_cases

        figure; hold on; grid on;

        for D = wheel_diameters
            DP_vals = zeros(size(wheel_widths));

            for i = 1:length(wheel_widths)
                DP_vals(i) = compute_DP(wheel_widths(i),D,N_w,slope,false,0,0);
            end

            plot(wheel_widths,DP_vals,'LineWidth',1.8, 'DisplayName',sprintf('Wheel Dia %.2f m',D));
        end

        xlabel('Wheel Width (m)');
        ylabel('Drawbar Pull (N)');
        title(sprintf('Drawbar Pull – No Grousers – %d Wheels – %d° Slope',N_w,slope));
        legend('Location','best');
    end
end

% 2) Grousers – Flat & 10° Slope
N_g_default = 12; % assumes 12 grousers
h_g_default = 0.02; % assumes grouser height of 2cm

for slope = [slope_flat slope_slope]
    for N_w = wheel_cases

        figure; hold on; grid on;

        for D = wheel_diameters
            DP_vals = zeros(size(wheel_widths));

            for i = 1:length(wheel_widths)
                DP_vals(i) = compute_DP(wheel_widths(i),D,N_w,slope,true,N_g_default,h_g_default);
            end

            plot(wheel_widths,DP_vals,'LineWidth',1.8, 'DisplayName',sprintf('Wheel Dia %.2f m',D));
        end

        xlabel('Wheel Width (m)');
        ylabel('Drawbar Pull (N)');
        title(sprintf('Drawbar Pull – Grousers – %d Wheels – %d° Slope',N_w,slope));
        legend('Location','best');
    end
end

% 3) Drawbar Pull – Effect of Number of Grousers
for N_w = wheel_cases
    for N_g = grouser_numbers

        figure; hold on; grid on;

        for D = wheel_diameters
            DP_vals = zeros(size(wheel_widths));

            for i = 1:length(wheel_widths)
                DP_vals(i) = compute_DP(wheel_widths(i),D,N_w,0,true,N_g,h_g_default);
            end

            plot(wheel_widths,DP_vals,'LineWidth',1.8, 'DisplayName',sprintf('Wheel Dia %.2f m',D));
        end

        xlabel('Wheel Width (m)');
        ylabel('Drawbar Pull (N)');
        title(sprintf('Drawbar Pull – %d Wheels – %d Grousers',N_w,N_g));
        legend('Location','best');
    end
end

% 4) Drawbar Pull – Effect of Grouser Height
N_g_fixed = 12; % assumes 12 grousers

for N_w = wheel_cases
    for h_g = grouser_heights

        figure; hold on; grid on;

        for D = wheel_diameters
            DP_vals = zeros(size(wheel_widths));

            for i = 1:length(wheel_widths)
                DP_vals(i) = compute_DP(wheel_widths(i),D,N_w,0,true,N_g_fixed,h_g);
            end

            plot(wheel_widths,DP_vals,'LineWidth',1.8, 'DisplayName',sprintf('Wheel Dia %.2f m',D));
        end

        xlabel('Wheel Width (m)');
        ylabel('Drawbar Pull (N)');
        title(sprintf('Drawbar Pull – %d Wheels – Grouser Height %.0f mm',N_w,h_g*1000));
        legend('Location','best');
    end
end

% 5) Sinkage Ratio – Flat & 10° Slope
for slope = [slope_flat slope_slope]
    for N_w = wheel_cases

        figure; hold on; grid on;

        for D = wheel_diameters
            z_ratio = zeros(size(wheel_widths));

            for i = 1:length(wheel_widths)
                z = compute_z(wheel_widths(i),D,N_w,slope);
                z_ratio(i) = z/D;
            end

            plot(wheel_widths,z_ratio,'LineWidth',1.8, 'DisplayName',sprintf('Wheel Dia %.2f m',D));
        end

        yline(0.05,'--','Bulldozing Limit','LineWidth',1.5);
        xlabel('Wheel Width (m)');
        ylabel('Sinkage Ratio (z / D)');
        title(sprintf('Sinkage Ratio – %d Wheels – %d° Slope',N_w,slope));
        legend('Location','best');
    end
end

% 6) Drawbar Pull to Weight Ratio
for slope = [slope_flat slope_slope]
    for N_w = wheel_cases

        figure; hold on; grid on;

        for D = wheel_diameters
            DPW_vals = zeros(size(wheel_widths));

            for i = 1:length(wheel_widths)
                DP = compute_DP(wheel_widths(i),D,N_w,slope,true,N_g_default,h_g_default);
                DPW_vals(i) = DP / (m_tot * g);
            end

            plot(wheel_widths,DPW_vals,'LineWidth',1.8, 'DisplayName',sprintf('Wheel Dia %.2f m',D));
        end

        xlabel('Wheel Width (m)');
        ylabel('Drawbar Pull / Weight');
        title(sprintf('DP/W – %d Wheels – %d° Slope',N_w,slope));
        legend('Location','best');
    end
end

% 7) Feasible Drawbar Pull – All Constraints Applied
for slope = [slope_flat slope_slope]

    % DP/W feasibility bounds
    if slope == slope_flat
        DPW_min = 0.20; DPW_max = 0.25;
    else
        DPW_min = 0.10; DPW_max = 0.25;
    end

    for N_w = wheel_cases
        figure; hold on; grid on;

        for D = wheel_diameters

            % Max rover length constraint
            if (N_w/2) * D > max_rover_length
                continue
            end

            DP_vals = nan(size(wheel_widths));

            for i = 1:length(wheel_widths)

                % Sinkage constraint (bulldozing avoidance)
                z = compute_z(wheel_widths(i),D,N_w,slope);
                if z/D >= 0.05
                    continue
                end

                % Drawbar pull and drawbar pull to weight ratio
                DP  = compute_DP(wheel_widths(i),D,N_w,slope,true,N_g_default,h_g_default);
                DPW = DP / (m_tot * g);

                if DPW >= DPW_min && DPW <= DPW_max
                    DP_vals(i) = DP;
                end
            end

            plot(wheel_widths,DP_vals,'LineWidth',1.8, 'DisplayName',sprintf('Wheel Dia %.2f m',D));
        end

        xlabel('Wheel Width (m)');
        ylabel('Drawbar Pull (N)');
        title(sprintf('Feasible Drawbar Pull – %d Wheels – %d° Slope',N_w,slope));
        legend('Location','best');
    end
end

% Drawbar Pull Function
function DP = local_drawbar_pull(b,D,N_w,theta_s,grousers,N_tot,h_g, m_tot,g,k_c,k_phi,n,c,gamma,phi,K_shear, K_c,K_gamma,N_c,N_q,N_gamma_t,s)

% Wheel loading
W_tot = m_tot * g;
W_w   = (W_tot * cosd(theta_s)) / N_w;

% Sinkage and contact length
z = ((3/(3-n))*(W_w/((k_c + b*k_phi)*sqrt(D))))^(2/(2*n+1));
l = (D/2)*acos(max(1 - 2*z/D, -1));

% Effective contact length with grousers
if grousers
    l_eff = l + h_g;
else
    l_eff = l;
end

% Compaction resistance
R_c = 0.85854*(W_w^4/((k_c+b*k_phi)*D^2))^(1/3);
R_c_total = N_w * R_c;

% Bulldozing resistance (conditional)
R_b_total = 0;
z_crit = 0.005; % 5mm critical sinkage level
s_crit = 0.5;

if (z > z_crit) && (s > s_crit)
    alpha = acos(max(1 - 2*z/D, -1));
    R_b = (b*sin(alpha+phi)/(2*sin(alpha)*cos(phi))) * (2*z*c*K_c + gamma*z^2*K_gamma);
    R_b_total = 2 * R_b * min(1,(s-s_crit)/(1-s_crit));
end

% Gravity and rolling resistance
R_g = W_tot * sind(theta_s);
R_r = (W_tot * cosd(theta_s)) * 0.01;

% Shear force generation
if grousers
    N_g = (N_tot * l_eff) / (pi * D);
    H = (b*l_eff*c*(1 + (2*h_g/b)*N_g) + W_w*tan(phi)*(1 + 0.64*(h_g/b)*atan(b/h_g))) * (1 - (K_shear/(s*l_eff))*(1 - exp(-s*l_eff/K_shear)));
else
    H = (b*l_eff*c + W_w*tan(phi)) * (1 - (K_shear/(s*l_eff))*(1 - exp(-s*l_eff/K_shear)));
end

H_total = N_w * H;

% Net drawbar pull
DP = H_total - (R_c_total + R_b_total + R_g + R_r);
end

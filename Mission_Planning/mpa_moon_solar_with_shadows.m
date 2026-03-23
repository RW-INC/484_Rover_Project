%% plots
% constants
n = 2831; % October 1, 2027
Tau = deg2rad(-1.545);
Y = 346.71;
t_0 = -1.23;
azi_p = deg2rad([0 180]); % 2-sided panels

T_total = 708.75; % lunar day hours
dt = 0.25;

t = 0:dt:T_total;
N = length(t);

R_moon = 1737400; % meters
h_panel = 0.3;    % meters

theta_dip = acos(R_moon/(R_moon + h_panel)); % radians

% preallocate
B = zeros(1,N);
azi_s = zeros(1,N);
del = zeros(1,N);
H = zeros(1,N);
theta_i = zeros(1,N);
G_act_over_mission = zeros(1,N);

% constants
L = deg2rad(-91); % local latitude 30 km from south pole
omega = 360/708.75;
E = deg2rad(90);
G_sc = 1361;
I = 0.3543; % found from image reader at region 7

for i = 1:N

    % declination
    del(i) = Tau*sin(deg2rad(360/Y*(n + t(i)/24 - t_0)));

    % hour angle
    H(i) = deg2rad(omega * (708.75/2 - t(i)));

    % elevation
    B(i) = asin(cos(L)*cos(del(i))*cos(H(i)) + sin(L)*sin(del(i)));

    % solar azimuth
    azi_s(i) = asin(cos(del(i))*sin(H(i))/cos(B(i)));

    % reset total flux
    G_total = 0;
    
    % check if Sun is above raised horizon
    if B(i) > -theta_dip
    
        for p = 1:2
    
            theta_i = acos(cos(B(i))*cos(azi_s(i)-azi_p(p)) + sin(B(i))*cos(E));
    
            G_panel = G_sc*cos(theta_i);
    
            if G_panel > 0
                G_total = G_total + G_panel;
            end
    
        end
    
    else
    
        G_total = 0;

end

% apply illumination factor
G_act_over_mission(i) = G_total * I;

end

% plots

figure;
plot(t, rad2deg(B));
xlabel('Time [hr]');
ylabel('Solar elevation angle [deg]');
title('Solar elevation angle vs Time');

figure;
plot(t, G_act_over_mission);
xlabel('Time [hr]');
ylabel('Solar flux [W/m^2]');
title('Solar flux vs Time');

figure;

% Left axis: Solar elevation angle
yyaxis left
plot(t, rad2deg(B), 'b', 'LineWidth', 2);
ylabel('Solar elevation angle [deg]');

% Right axis: Solar flux
yyaxis right
plot(t, G_act_over_mission, 'r', 'LineWidth', 2);
ylabel('Solar flux [W/m^2]');

% Common x-axis
xlabel('Time [hr]');
title('Solar elevation angle and Solar flux vs Time');

grid on;
legend('Solar elevation angle', 'Solar flux');

G_avg = mean(G_act_over_mission);

disp(['Average solar flux over lunar day: ', num2str(G_avg), ' W/m^2'])

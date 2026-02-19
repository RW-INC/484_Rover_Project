% solve for del
Tau = deg2rad(-1.545); %[deg]
Y = 346.71; % [earth-days]
t_0 = -1.23; % [earth-days]
n = 2831; % (October 1, 2027) day number, with n = 1 representing the date of the beginning of the data, that is Jan 1st, 2020
t = 0; % time variable representing the time of the lunar day in hours, t = 0 local noon, 0 ≤ 𝑡 < 708.75 ℎ𝑟
del = Tau*sin(deg2rad(360/Y*(n + t/24 - t_0))); % lunar declination angle

% solve for B
L = deg2rad(85); % local latitude, 80deg to 90deg on south pole 
H = deg2rad(0.515*(354.365 - t)); % lunar hour angle
B = asin(cos(L)*cos(del)*cos(H) + sin(L)*sin(del)); % solar elevation angle

E = deg2rad(90); % panel tilt angle
B_guess = deg2rad(0); % solar elevation angle estimate
azi_s = asin(cos(del)*sin(H)/cos(B)); % solar azimuth
azi_p = 0; % azimuth of the surface horizontal orientation 

theta_i = acos(cos(B)*cos(azi_s-azi_p) + sin(B)*cos(E)); % incidence angle on panel
theta_i_deg = rad2deg(theta_i)

G_sc = 1361; % Mean solar constant at lunar distance
I = 0.695; % illumination average worse case from NASA LOLA
G_act = G_sc*cos(theta_i)*I % W/m^2

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

% preallocate
B = zeros(1,N);
azi_s = zeros(1,N);
del = zeros(1,N);
H = zeros(1,N);
theta_i = zeros(1,N);
G_act_over_mission = zeros(1,N);

% constants
L = deg2rad(-85);
omega = 360/708.75;
E = deg2rad(90);
G_sc = 1361;
I = 0.695;

for i = 1:N

    % declination
    del(i) = Tau*sin(deg2rad(360/Y*(n + t(i)/24 - t_0)));

    % hour angle
    H(i) = deg2rad(omega * (708.75/2 - t(i)));

    % elevation
    B(i) = asin(cos(L)*cos(del(i))*cos(H(i)) + sin(L)*sin(del(i)));

    % solar azimuth
    azi_s(i) = asin(cos(del(i))*sin(H(i))/cos(B(i)));

    % reset total flux at this time step
    G_total = 0;

    % sum contribution from each panel
    for p = 1:2

        theta_i = rad2deg(acos(cos(B(i))*cos(azi_s(i)-azi_p(p)) + sin(B(i))*cos(E)));

        G_panel = G_sc*cos(deg2rad(theta_i));

        % prevent negative flux from backside
        if G_panel > 0
            G_total = G_total + G_panel;
        end

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

%% plot with solar iteration

% constants
n = 2831; % (October 1, 2027) day number, with n = 1 representing the date of the beginning of the data, that is Jan 1st, 2020
Tau = deg2rad(-1.545); %[deg]
Y = 346.71; % [earth-days]
t_0 = -1.23; % [earth-days]

T_total = 708.75; % lunar day hours
dt = 0.25; % timestep hours

t = 0:dt:T_total;

N = length(t);

B = zeros(1,N);
azi_s = zeros(1,N);

% shadow percentages

shadow_percent = 0:10:100;
illum_factor = 1 - shadow_percent/100;

% prepare plot

figure;
hold on;
grid on;

colors = lines(length(shadow_percent));

legend_entries = strings(length(shadow_percent),1);

% loop over shadow levels

for s = 1:length(shadow_percent)

    I = illum_factor(s);

    G_act = zeros(1,N);

    for i = 1:N
        del = Tau*sin(deg2rad(360/Y*(n + t(i)/24 - t_0))); % lunar declination angle
        
        % solve for B
        L = deg2rad(-85); % local latitude, -80deg to -90deg on south pole 
        omega = 360/708.75; % deg/hr
        H = deg2rad(omega * (708.75/2 - t(i))); % lunar hour angle
        B(i) = asin(cos(L)*cos(del)*cos(H) + sin(L)*sin(del)); % solar elevation angle
    
        E = deg2rad(90); % panel tilt angle
        azi_s(i) = asin(cos(del)*sin(H)/cos(B(i))); % solar azimuth
        azi_p = 0; % azimuth of the surface horizontal orientation 
    
        theta_i(i) = rad2deg(acos(cos(B(i))*cos(azi_s(i)-azi_p) + sin(B(i))*cos(E))); % incidence angle on panel
        
        G_sc = 1361; % Mean solar constant at lunar distance
        I = 0.695; % illumination average worse case from NASA LOLA
        G_act_over_mission(i) = G_sc*cos(deg2rad(theta_i(i)))*I;

        G_act(i) = G_sc * max(0, cos(deg2rad(theta_i)) * I;

    end

    plot(t, G_act, 'LineWidth', 1.5, 'Color', colors(s,:));

    legend_entries(s) = sprintf('%d%% shadow', shadow_percent(s));

end

xlabel('Time [hr]');
ylabel('Solar Flux [W/m^2]');
title('Solar Flux vs Time for 0–100% Shadow Conditions');
legend(legend_entries, 'Location', 'bestoutside');

hold off;

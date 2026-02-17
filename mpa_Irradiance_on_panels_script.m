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
G_act = G_sc*cos(theta_i)*0.7 % W/m^2


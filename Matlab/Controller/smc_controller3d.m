%% Clear workspace
clear all;
clc;
close all;

%% Setup controller parameters
K = [0.02 ; 0.02 ; 1.0];
eps_v = [5; 5; 5];

%% Setup rover geometry 
geom.r = 0.17/2;
geom.B = 0.2;
geom.L = 0.25;
geom.max_w = 1.0;

%% Setup simulation parameters
f_physics = 1e2;
f_control = 20;
dt = 1/f_physics;
decimation = f_physics/f_control;
noise_scales = [0.01; 0.01; 0.05] * 0;

% Physical reality slip parameters
mu_R_phys = @(wr) 0.98;
mu_L_phys = @(wl) 0.98;

% Controller's internal assumed slip parameters
mu_R_ctrl = @(mu_R_phys) 1.0;
mu_L_ctrl = @(mu_L_phys) 1.0;

%% Symbolic Jacobian Setup
syms theta_sym w_r_sym w_l_sym mu_R_sym mu_L_sym B_track_sym r_w_sym real

v_sym = (r_w_sym * mu_R_sym / 2) * w_r_sym + (r_w_sym * mu_L_sym / 2) * w_l_sym;
omega_sym = (r_w_sym * mu_R_sym / B_track_sym) * w_r_sym - (r_w_sym * mu_L_sym / B_track_sym) * w_l_sym;
H_sym = [v_sym * cos(theta_sym); v_sym * sin(theta_sym); omega_sym];

J_u_sym = jacobian(H_sym, [w_r_sym, w_l_sym]);

calc_J = matlabFunction(J_u_sym, 'Vars', {theta_sym, w_r_sym, w_l_sym, mu_R_sym, mu_L_sym, B_track_sym, r_w_sym});
calc_H = matlabFunction(H_sym, 'Vars', {theta_sym, w_r_sym, w_l_sym, mu_R_sym, mu_L_sym, B_track_sym, r_w_sym});

%% Setup reference trajectory
t_knots = 0:15:100;
testcase = Testcase(0, 1000, f_physics, t_knots, 0.01, 0.022);
Traj = testcase.traj;
n_steps = length(testcase.traj.t_master);

% testcase.plot_scenario();

t_X  = Traj.X;       t_Y  = Traj.Y;       t_Th  = Traj.Theta;
t_Xd = Traj.X_dot;   t_Yd = Traj.Y_dot;   t_Thd = Traj.Theta_dot;

%% Buffers
x_hist = zeros(3, n_steps);
u_hist = zeros(4, n_steps);
s_hist = zeros(3, n_steps);

%% Simulation loop setup
x_state = [t_X(1) ; t_Y(1); mod(t_Th(1), 2*pi)];

full_state = [
    x_state(1);             % 1. X
    x_state(2);             % 2. Y
    testcase.Z_traj(1);     % 3. Z
    0;                      % 4. Vx
    0;                      % 5. Vy
    0;                      % 6. Vz
    0;                      % 7. Roll
    0;                      % 8. Pitch
    x_state(3)              % 9. Yaw (Starting heading)
];
u_state = [0;0;0;0];

% codegen fullstatedynamics, cuz its slow.
% clear FullStateDynamicsInC_mex
codegen FullStateDynamicsInC.m -args {full_state, u_state, testcase.xmin, testcase.xmin, testcase.dx, testcase.dx, testcase.Z_map, geom, dt, []} 
%% Simulation loop
for i = 1:n_steps
    ref_X  = t_X(i);
    ref_Y  = t_Y(i);
    ref_Th = mod(t_Th(i), 2*pi);
    
    if mod(i-1, decimation) == 0
        ref_Xd  = t_Xd(i);
        ref_Yd  = t_Yd(i);
        ref_Thd = t_Thd(i);
        
        s_1 = x_state(1) - ref_X;
        s_2 = x_state(2) - ref_Y;
        s_3 = mod(x_state(3) - ref_Th + pi, 2*pi) - pi;
        s = [s_1; s_2; s_3];
        
        current_J = calc_J(x_state(3), u_state(1), u_state(2), mu_R_ctrl(0), mu_L_ctrl(0), geom.B, geom.r);
        current_H = calc_H(x_state(3), u_state(1), u_state(2), mu_R_ctrl(0), mu_L_ctrl(0), geom.B, geom.r);
        
        sig_s = tanh(s ./ (2 .* eps_v));
        ref_dot = [ref_Xd; ref_Yd; ref_Thd];
       
        lambda = 5;

        delta_u = pinv(current_J) * (-K .* sig_s - current_H + ref_dot - lambda * s);
        
        u_state(1) = clip(u_state(1) + delta_u(1), -geom.max_w, geom.max_w);
        u_state(2) = clip(u_state(2) + delta_u(2), -geom.max_w, geom.max_w);
        u_state(3) = delta_u(1);
        u_state(4) = delta_u(2);

        s_hist(:,i) = s;
    else
        u_state(3) = 0;
        u_state(4) = 0;

        s_hist(:,i) = s_hist(:, max(i-1, 1));
    end
    
    uR = mu_R_phys(u_state(1)) * u_state(1);
    uL = mu_L_phys(u_state(2)) * u_state(2);
    

    full_state = full_state + (FullStateDynamicsInC_mex(full_state, ...
        u_state, ...
        testcase.xmin, ...
        testcase.xmin, ...
        testcase.dx, ...
        testcase.dx, ...
        testcase.Z_map, ...
        geom, ...
        dt, ...
        []) + 0.01^2 * randn(size(full_state)))* dt;

    x_state = [full_state(1:2); full_state(end)];
    x_hist(:,i) = x_state;
    u_hist(:,i) = u_state;
end

% profile viewer;close(f);
%% 7. Plots
t = Traj.t_master;

figure('Name', 'Trajectory', 'Position', [200 200 700 600]);
ax = subplot(1,1,1);
testcase.plot_scenario(ax);

% plot(t_X, t_Y, 'k--', 'LineWidth', 2); hold on;
plot3(x_hist(1,:), x_hist(2,:), testcase.Z_traj, 'b', 'LineWidth', 1.2, ...
    'DisplayName','Actual Path');
% plot(t_X(1), t_Y(1), 'k^', 'MarkerFaceColor', 'k');
axis equal; grid on;
legend;
xlabel('X (m)'); ylabel('Y (m)'); title('Path');

figure('Name', 'Control History', 'Position', [100 100 1000 900]);

subplot(3,1,1);
plot(t, u_hist(1,:), 'r', 'LineWidth', 1); hold on;
plot(t, u_hist(2,:), 'b', 'LineWidth', 1);
yline(geom.max_w, 'k--'); yline(-geom.max_w, 'k--');
grid on; legend('\omega_R', '\omega_L', 'Saturation');
ylabel('Wheel speed (rad/s)'); title('Control Input');

subplot(3,1,2);
plot(t, s_hist(1,:), 'r', t, s_hist(2,:), 'b', t, s_hist(3,:), 'g', 'LineWidth', 1);
grid on; legend('s_1 (X)', 's_2 (Y)', 's_3 (\theta)');
ylabel('Sliding variable'); title('Sliding Variables');

subplot(3,1,3);
N_fft = 2^nextpow2(n_steps);
f_axis = (0:N_fft/2-1) * f_physics / N_fft;
U1_fft = abs(fft(u_hist(1,:), N_fft));
U2_fft = abs(fft(u_hist(2,:), N_fft));
plot(f_axis, U1_fft(1:N_fft/2), 'r', f_axis, U2_fft(1:N_fft/2), 'b', 'LineWidth', 1);
xlim([0 f_control]);
xline(f_control/2, 'k--', 'Nyquist');
grid on; legend('|\omega_R(f)|', '|\omega_L(f)|');
xlabel('Frequency (Hz)'); ylabel('Magnitude'); title('Control Spectrum (chatter diagnostic)');
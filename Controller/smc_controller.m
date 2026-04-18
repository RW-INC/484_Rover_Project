function smc_controller()
clear; clc; close all;

%% 1. Physics Parameters (match driver)
K = [0.01; 0.01; 0.05]*2;

r = 0.17/2; B = 0.2;
max_w = 0.235;

f_phys = 1e4;
f_ctrl = 100;
dt = 1/f_phys;
decim = f_phys/f_ctrl;

noise_scales = [0.01 ; 0.01; 0.05];
eps_v = 0.05;

disp(eps_v)

mu_R = 0.98;
mu_L = 0.98;

%% 2. Reference Trajectory
t_knots = 0:75:250;
Traj = Trajectory(f_phys, t_knots, 0, 0, 0.00, 0.012);
n_steps = length(Traj.t_master);

t_X  = Traj.X;       t_Y  = Traj.Y;       t_Th  = Traj.Theta;
t_Xd = Traj.X_dot;   t_Yd = Traj.Y_dot;   t_Thd = Traj.Theta_dot;

%% 3. Precomputed constants
inv_r    = 1 / r;
r_over_2 = r / 2;
r_over_B = r / B;
half_B   = B / 2;
two_pi   = 2 * pi;

K_1 = K(1); K_2 = K(2); K_3 = K(3);
inv_2_epsv = 1 / (2 * eps_v);

ns_1 = noise_scales(1)^2;
ns_2 = noise_scales(2)^2;
ns_3 = noise_scales(3)^2;

%% 4. Initial state
th0 = t_Th(1);
while th0 >= two_pi, th0 = th0 - two_pi; end
while th0 < 0,       th0 = th0 + two_pi; end

x_state = [t_X(1); t_Y(1); th0];
u_state = [0; 0];

%% 5. History buffers
x_hist = zeros(3, n_steps);
u_hist = zeros(2, n_steps);
s_hist = zeros(3, n_steps);
L_hist = zeros(3, n_steps);

%% 6. Time loop
for i = 1:n_steps
    update_ctrl = (mod(i - 1, decim) == 0);

    ref_X  = t_X(i);
    ref_Y  = t_Y(i);
    ref_Th = t_Th(i);
    while ref_Th >= two_pi, ref_Th = ref_Th - two_pi; end
    while ref_Th < 0,       ref_Th = ref_Th + two_pi; end

    ref_Xd = 0; ref_Yd = 0; ref_Thd = 0;
    
    if update_ctrl
        ref_Xd  = t_Xd(i);
        ref_Yd  = t_Yd(i);
        ref_Thd = t_Thd(i);
    end

    c_th = cos(x_state(3));
    s_th = sin(x_state(3));

    if update_ctrl
        s_1 = x_state(1) - ref_X;
        s_2 = x_state(2) - ref_Y;
        s_3 = x_state(3) - ref_Th;
        while s_3 > pi,  s_3 = s_3 - two_pi; end
        while s_3 < -pi, s_3 = s_3 + two_pi; end

        v_nom = r_over_2 * (u_state(1) + u_state(2));
        w_nom = r_over_B * (u_state(1) - u_state(2));

        L_1 = -K_1 * tanh(s_1 * inv_2_epsv) - v_nom * c_th + ref_Xd;
        L_2 = -K_2 * tanh(s_2 * inv_2_epsv) - v_nom * s_th + ref_Yd;
        L_3 = -K_3 * tanh(s_3 * inv_2_epsv) - w_nom + ref_Thd;

        L_proj = L_1 * c_th + L_2 * s_th;

        u_state(1) = max(min(u_state(1) + (L_proj + L_3 * half_B) * inv_r, max_w), -max_w);
        u_state(2) = max(min(u_state(2) + (L_proj - L_3 * half_B) * inv_r, max_w), -max_w);

        s_hist(:, i) = [s_1; s_2; s_3];
        L_hist(:, i) = [L_1; L_2; L_3];
    else
        s_hist(:, i) = s_hist(:, max(i-1, 1));
        L_hist(:, i) = L_hist(:, max(i-1, 1));
    end

    noise = randn(3, 1);
    uR = mu_R * u_state(1);
    uL = mu_L * u_state(2);

    x1 = x_state(1) + (r_over_2 * (uR + uL) * c_th + ns_1 * noise(1)) * dt;
    x2 = x_state(2) + (r_over_2 * (uR + uL) * s_th + ns_2 * noise(2)) * dt;
    x3 = x_state(3) + (r_over_B * (uR - uL) + ns_3 * noise(3)) * dt;
    while x3 >= two_pi, x3 = x3 - two_pi; end
    while x3 < 0,       x3 = x3 + two_pi; end

    x_state = [x1; x2; x3];

    x_hist(:, i) = x_state;
    u_hist(:, i) = u_state;
end

%% 7. Plots
t = Traj.t_master;

figure('Name', 'Trajectory', 'Position', [200 200 700 600]);
plot(t_X, t_Y, 'k--', 'LineWidth', 2); hold on;
plot(x_hist(1,:), x_hist(2,:), 'b', 'LineWidth', 1.2);
plot(t_X(1), t_Y(1), 'k^', 'MarkerFaceColor', 'k');
axis equal; grid on;
legend('Reference', 'Actual', 'Start');
xlabel('X (m)'); ylabel('Y (m)'); title('Path');

figure('Name', 'Control History', 'Position', [100 100 1000 900]);

subplot(4,1,1);
plot(t, u_hist(1,:), 'r', 'LineWidth', 1); hold on;
plot(t, u_hist(2,:), 'b', 'LineWidth', 1);
yline(max_w, 'k--'); yline(-max_w, 'k--');
grid on; legend('\omega_R', '\omega_L', 'Saturation');
ylabel('Wheel speed (rad/s)'); title('Control Input');

subplot(4,1,2);
plot(t, s_hist(1,:), 'r', t, s_hist(2,:), 'b', t, s_hist(3,:), 'g', 'LineWidth', 1);
grid on; legend('s_1 (X)', 's_2 (Y)', 's_3 (\theta)');
ylabel('Sliding variable'); title('Sliding Variables');

subplot(4,1,3);
du1 = diff(u_hist(1,:)) / dt;
du2 = diff(u_hist(2,:)) / dt;
plot(t(2:end), du1, 'r', t(2:end), du2, 'b', 'LineWidth', 0.5);
grid on; legend('d\omega_R/dt', 'd\omega_L/dt');
ylabel('Rate (rad/s^2)'); title('Control Rate (chatter indicator)');

subplot(4,1,4);
N_fft = 2^nextpow2(n_steps);
f_axis = (0:N_fft/2-1) * f_phys / N_fft;
U1_fft = abs(fft(u_hist(1,:), N_fft));
U2_fft = abs(fft(u_hist(2,:), N_fft));
plot(f_axis, U1_fft(1:N_fft/2), 'r', f_axis, U2_fft(1:N_fft/2), 'b', 'LineWidth', 1);
xlim([0 f_ctrl]);
xline(f_ctrl/2, 'k--', 'Nyquist');
grid on; legend('|\omega_R(f)|', '|\omega_L(f)|');
xlabel('Frequency (Hz)'); ylabel('Magnitude'); title('Control Spectrum (chatter diagnostic)');

end
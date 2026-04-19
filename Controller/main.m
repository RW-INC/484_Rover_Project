close all; clear all; clc;
addpath 'C:\Users\srikr\Desktop\SPARX\Nav\Full State Estimation'
addpath 'C:\Users\srikr\Desktop\SPARX\Controller'

patch_size = 10;        
N_res = 200;            
t_vec = 0:300:2000;       

% === SPEED KNOBS ===
speed_factor = 10;
dt = 0.001;
target_fps = 60;

estimation_freq = 100;
ctrl_freq = 20;
sim_freq = 1 / dt;
steps_per_frame = max(1, round(speed_factor / (dt * target_fps)));
% ===================

my_test = Testcase(patch_size, N_res, sim_freq, t_vec, 0, 0.01);

geom.r = 0.17/2;
geom.B = 0.2;
geom.L = 0.25;

mission_time = 0;   
u = [0; 0; 0; 0]; 
max_w = 0.935;

lander_pos = [my_test.traj.X(1); my_test.traj.Y(1); my_test.Z_traj(1)];
curr_state = [lander_pos; 0; 0; 0; my_test.traj.Theta(1); 0; 0];
curr_state(7) = mod(curr_state(7), 2*pi);

scale = 0.8;
sun_scale = 12;

% =============================================
% STATE ESTIMATION INITIALIZATION
% =============================================
sigma_accel  = 0.03;
sigma_gyro   = 0.15 * pi/180 / 60;
sigma_pos    = 0.03;
sigma_vel    = 0.003;
sigma_rho    = 0.01;
sigma_rhodot = 0.001;
sigma_point  = 0.01;

bias_rate_gyro = 0.5 * pi/180 / 60;

yaw0 = curr_state(7);
q0 = [cos(yaw0/2), 0, 0, sin(yaw0/2)];  
mekf_state = [q0, 0, 0, 0];              
mekf_P = blkdiag(eye(3)*0.01, eye(3)*0.001);

trans_state = [lander_pos; 0; 0; 0];
trans_P = diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01]);

R_est = eye(3);

% =============================================
% LIVE PLOTTING BUFFERS
% =============================================
ekg_len = 500;
t_buf  = nan(1, ekg_len);

wR_buf = nan(1, ekg_len);
wL_buf = nan(1, ekg_len);
s1_buf = nan(1, ekg_len);
s2_buf = nan(1, ekg_len);
s3_buf = nan(1, ekg_len);

% Actual slip (from model: 0.75 + 0.23*exp(-|w|))
mu_r_act_buf = nan(1, ekg_len);
mu_l_act_buf = nan(1, ekg_len);
% Estimated slip (from kinematics: 1 - r*w/v)
mu_r_est_buf = nan(1, ekg_len);
mu_l_est_buf = nan(1, ekg_len);

pos_true_buf = nan(3, ekg_len); pos_est_buf = nan(3, ekg_len);
vel_true_buf = nan(3, ekg_len); vel_est_buf = nan(3, ekg_len);
att_true_buf = nan(3, ekg_len); att_est_buf = nan(3, ekg_len);

hist_time     = [];
hist_true_pos = []; hist_est_pos  = [];
hist_true_vel = []; hist_est_vel  = [];
hist_true_att = []; hist_est_att  = [];

% Hold values between controller updates
mu_r_act_hold = NaN;
mu_l_act_hold = NaN;
mu_r_est_hold = NaN;
mu_l_est_hold = NaN;

% =============================================
% GET SCREEN SIZE FOR FULLSCREEN SPLIT LAYOUT
% =============================================
scrsz = get(0, 'ScreenSize');
SW = scrsz(3);
SH = scrsz(4);

% =============================================
% FIGURE 1: 3D ENVIRONMENT (Left Half of Screen)
% =============================================
fig_sim = figure('Name', 'Rover Simulation', 'Position', [1, 1, SW/2, SH], 'Color', 'k');
ax3d = subplot(1, 1, 1);
my_test.plot_scenario(ax3d);

hSun = quiver3(lander_pos(1), lander_pos(2), lander_pos(3), 0,0,0, 0, 'Color', [1 1 0], 'LineWidth', 2, 'AutoScale', 'off');
hSunPt = plot3(0,0,0, 'yo', 'MarkerFaceColor', 'y', 'MarkerSize', 8);
hFwd = quiver3(0,0,0,0,0,0, 0, 'r', 'LineWidth', 2, 'AutoScale', 'off'); 
hLft = quiver3(0,0,0,0,0,0, 0, 'g', 'LineWidth', 2, 'AutoScale', 'off'); 
hUp  = quiver3(0,0,0,0,0,0, 0, 'b', 'LineWidth', 2, 'AutoScale', 'off');
hRover = plot3(lander_pos(1), lander_pos(2), lander_pos(3), 'wo', 'MarkerSize', 6, 'MarkerFaceColor', 'w');
hRover_quiver = quiver3(lander_pos(1), lander_pos(2), lander_pos(3), ...
    curr_state(1), curr_state(2), curr_state(3), 'o', 'LineWidth', 2, 'AutoScale', 'off');
hRover_sun = quiver3(curr_state(1), curr_state(2), curr_state(3),0,0,0, 'o', 'LineWidth', 2,'AutoScale', 'off');
h_attitude = quiver3(curr_state(1), curr_state(2), curr_state(3),0,0,0, 'w', 'LineWidth', 2,'AutoScale', 'off');

% =============================================
% FIGURE 2: CONTROLLER INPUTS (Top Right Quarter)
% =============================================
fig_ctrl = figure('Name', 'Controller Inputs', 'Position', [SW/2 + 1, SH/2 + 1, SW/2, SH/2], 'Color', 'k');

ax_ekg = subplot(3, 1, 1);
set(ax_ekg, 'Color', 'k', 'XColor', 'g', 'YColor', 'g', 'GridColor', [0.1 0.4 0.1]);
hold on; grid on;
hWR = plot(ax_ekg, t_buf, wR_buf, 'r', 'LineWidth', 1.5);
hWL = plot(ax_ekg, t_buf, wL_buf, 'c', 'LineWidth', 1.5);
yline(ax_ekg, max_w, 'g--', 'LineWidth', 0.8);
yline(ax_ekg, -max_w, 'g--', 'LineWidth', 0.8);
ylabel(ax_ekg, '\omega (rad/s)', 'Color', 'g');
title(ax_ekg, 'WHEEL SPEED', 'Color', 'g', 'FontName', 'Courier');
legend(ax_ekg, {'\omega_R', '\omega_L'}, 'TextColor', 'g', 'Color', 'k', 'EdgeColor', 'g');
ylim(ax_ekg, [-max_w*1.3, max_w*1.3]);

ax_err = subplot(3, 1, 2);
set(ax_err, 'Color', 'k', 'XColor', 'g', 'YColor', 'g', 'GridColor', [0.1 0.4 0.1]);
hold on; grid on;
hS1 = plot(ax_err, t_buf, s1_buf, 'r', 'LineWidth', 1.5);
hS2 = plot(ax_err, t_buf, s2_buf, 'c', 'LineWidth', 1.5);
hS3 = plot(ax_err, t_buf, s3_buf, 'm', 'LineWidth', 1.5);
yline(ax_err, 0, 'g--', 'LineWidth', 0.5);
ylabel(ax_err, 'Error', 'Color', 'g');
title(ax_err, 'TRACKING ERROR', 'Color', 'g', 'FontName', 'Courier');
legend(ax_err, {'Position Error', 'Velocity Error', 'Yaw Error'}, 'TextColor', 'g', 'Color', 'k', 'EdgeColor', 'g');

ax_mu = subplot(3, 1, 3);
set(ax_mu, 'Color', 'k', 'XColor', 'g', 'YColor', 'g', 'GridColor', [0.1 0.4 0.1]);
hold on; grid on;
hMuRact = plot(ax_mu, t_buf, mu_r_act_buf, 'r-',  'LineWidth', 1.5);
hMuRest = plot(ax_mu, t_buf, mu_r_est_buf, 'r--', 'LineWidth', 1.5);
hMuLact = plot(ax_mu, t_buf, mu_l_act_buf, 'c-',  'LineWidth', 1.5);
hMuLest = plot(ax_mu, t_buf, mu_l_est_buf, 'c--', 'LineWidth', 1.5);
ylabel(ax_mu, '\mu', 'Color', 'g');
xlabel(ax_mu, 'Time (s)', 'Color', 'g');
title(ax_mu, 'SLIP RATIO', 'Color', 'g', 'FontName', 'Courier');
legend(ax_mu, {'\mu_R', '\mu_{R,est}', '\mu_L', '\mu_{L,est}'}, ...
    'TextColor', 'g', 'Color', 'k', 'EdgeColor', 'g');
ylim(ax_mu, [0, 1.1]);

% =============================================
% FIGURE 3: LIVE FULL 9-DOF STATE EST (Bottom Right Quarter)
% =============================================
fig_live_est = figure('Name', 'Live Full State Estimation', 'Position', [SW/2 + 1, 1, SW/2, SH/2], 'Color', 'k');

ax_handles = gobjects(3,3);
h_true     = gobjects(3,3);
h_est      = gobjects(3,3);

labels = {'X (m)', 'Vx (m/s)', 'Yaw (rad)';
          'Y (m)', 'Vy (m/s)', 'Pitch (rad)';
          'Z (m)', 'Vz (m/s)', 'Roll (rad)'};

titles = {'POS X', 'VEL X', 'YAW';
          'POS Y', 'VEL Y', 'PITCH';
          'POS Z', 'VEL Z', 'ROLL'};

for row = 1:3
    for col = 1:3
        ax_handles(row, col) = subplot(3, 3, (row-1)*3 + col);
        set(ax_handles(row, col), 'Color', 'k', 'XColor', 'g', 'YColor', 'g', 'GridColor', [0.1 0.4 0.1]); 
        hold on; grid on;
        h_true(row, col) = plot(ax_handles(row, col), t_buf, nan(1, ekg_len), 'c-', 'LineWidth', 2);
        h_est(row, col)  = plot(ax_handles(row, col), t_buf, nan(1, ekg_len), 'r--', 'LineWidth', 1.5);
        ylabel(ax_handles(row, col), labels{row, col}, 'Color', 'g');
        title(ax_handles(row, col), titles{row, col}, 'Color', 'g', 'FontName', 'Courier');
        if row == 3, xlabel(ax_handles(row, col), 'Time (s)', 'Color', 'g'); end
        if row == 1 && col == 1
            legend(ax_handles(row, col), {'Truth', 'Estimate'}, 'TextColor', 'g', 'Color', 'k', 'EdgeColor', 'g', 'Location', 'best');
        end
    end
end

% =============================================
% MAIN SIMULATION LOOP
% =============================================
i = 0;
n_traj = length(my_test.traj.X);
boundary_hit = false;

while ~boundary_hit
    for sub = 1:steps_per_frame
        i = i + 1;
        mission_time = mission_time + dt;

        if i >= n_traj
            boundary_hit = true;
            break;
        end

        % =============================================
        % 1. PLANT: propagate true state
        % =============================================
        dstate = FullStateDynamics(curr_state, u, my_test, geom);
        curr_state = curr_state + dt * dstate;
        curr_state(7) = mod(curr_state(7), 2*pi);

        yaw_true   = curr_state(7); 
        pitch_true = curr_state(8); 
        roll_true  = curr_state(9);

        R_true = [cos(yaw_true) -sin(yaw_true) 0; sin(yaw_true) cos(yaw_true) 0; 0 0 1] * ...
                 [cos(pitch_true) 0 sin(pitch_true); 0 1 0; -sin(pitch_true) 0 cos(pitch_true)] * ...
                 [1 0 0; 0 cos(roll_true) -sin(roll_true); 0 sin(roll_true) cos(roll_true)];

        % =============================================
        % 3. SIMULATE SENSORS
        % =============================================
        a_true = dstate(4:6);
        g_lunar = [0; 0; -1.625];
        a_imu = R_true' * (a_true - g_lunar) + randn(3,1) * sigma_accel;

        w_true = dstate(7:9);
        w_imu = w_true + bias_rate_gyro * dt + randn(3,1) * sigma_gyro;

        pos_rel = curr_state(1:3) - lander_pos;
        vel_true = curr_state(4:6);
        rho_true = norm(pos_rel);
        rhodot_true = dot(pos_rel, vel_true) / max(rho_true, 1e-6);
        uwb_rho = rho_true + randn * sigma_rho;
        uwb_rhodot = rhodot_true + randn * sigma_rhodot;

        days_elapsed = mission_time / (24 * 3600);
        s_dir = LanderSunPointingVector(2831, days_elapsed);

        imu_pos = curr_state(1:3) + randn(3,1) * sigma_pos;
        imu_vel = curr_state(4:6) + randn(3,1) * sigma_vel;

        % =============================================
        % 4. ATTITUDE ESTIMATION (TRIAD + MEKF)
        % =============================================
        mekf_P_saved = [];
        trans_P_saved = [];

        if mod(i-1, sim_freq/estimation_freq) == 0
            noisy_angles = randn(3,1) * sigma_point;
            noisy_quaternion = quaternion(noisy_angles', 'rotvec');
            
            g_body = -a_imu / norm(a_imu);
            R_r_sun_est = R_true' * s_dir;
            R_r_sun_est = rotatepoint(noisy_quaternion, R_r_sun_est' / norm(R_r_sun_est))';
            R_r_sun_est = R_r_sun_est/norm(R_r_sun_est);
    
            L_r_sun = s_dir / norm(s_dir);
            R_triad = TRIAD(L_r_sun, R_r_sun_est, g_body);
            q_triad_obj = quaternion(R_triad, 'rotmat', 'frame');
    
            w_imu_mod = w_imu - [0; 0; 0.085 * (u(1) - u(2)) / 0.2];
    
            [mekf_state, mekf_P] = rotational_mekf_fixed(...
                q_triad_obj, w_imu_mod, mekf_state, mekf_P, u, dt, ...
                sigma_gyro);
        
            q_est_arr = mekf_state(1:4);
            q_est_obj = quaternion(q_est_arr(1), q_est_arr(2), q_est_arr(3), q_est_arr(4));
            R_est = rotmat(q_est_obj, 'frame');
            
            yaw_est   = atan2(R_est(2,1), R_est(1,1));
            pitch_est = asin(-R_est(3,1));
            roll_est  = atan2(R_est(3,2), R_est(3,3));
            
            yaw_est = mod(yaw_est, 2*pi);
            pitch_est = mod(pitch_est + pi, 2*pi) - pi;
            roll_est = mod(roll_est, 2*pi);

        % =============================================
        % 5. TRANSLATIONAL EKF
        % =============================================
            imu_pos_rel = imu_pos - lander_pos;
    
            [trans_state, trans_P] = translational_ekf_fixed(...
                [imu_pos_rel; imu_vel], [uwb_rho; uwb_rhodot], ...
                [roll_est;pitch_est;yaw_est], u, trans_state, trans_P, mekf_P, dt);
    
            pos_est = trans_state(1:3) + lander_pos;
            vel_est = trans_state(4:6);
    
            if any(isnan(yaw_est)) || ... 
                any(isnan(pitch_est)) || ...
                any(isnan(roll_est)) || ...
                any(isnan(pos_est)) || ...
                any(isnan(vel_est))
                beep;
                pause;
            end

            mekf_P_saved = mekf_P;
            trans_P_saved = trans_P;
        end

        % =============================================
        % 6. CONTROLLER 
        % =============================================
        xd = [my_test.traj.X(i), my_test.traj.Y(i), my_test.traj.Theta(i), ...
              my_test.traj.X_dot(i), my_test.traj.Y_dot(i), my_test.traj.Theta_dot(i)]';

        if mod(i - 1, sim_freq/ctrl_freq) == 0
            q = diag(mekf_P_saved(1:3,1:3));
            q = quaternion([sqrt(1 - sum(q.^2)) ; q]');
            eulers = quat2eul(q,'XYZ');
            state_P = diag([trans_P_saved(1,1); trans_P_saved(2,2); eulers(3)^2]);
            
            v = norm(vel_est) + 1e-6;
            mu_r = abs(geom.r * u(1)/(v*cos(yaw_est)));
            mu_l = abs(geom.r * u(2)/(v*cos(yaw_est)));
            mu_est = [mu_r; mu_l]

            % Actual slip model
            mu_r_act_hold = 0.75 + 0.23 * exp(-abs(u(1)));
            mu_l_act_hold = 0.75 + 0.23 * exp(-abs(u(2)));
            % Estimated slip
            mu_r_est_hold = mu_r;
            mu_l_est_hold = mu_l;

            u = SMC([pos_est(1); pos_est(2); yaw_est], xd, u, geom, max_w, state_P, mu_est);
        else
            u(3:4) = [0;0];
        end

        % --- Boundary check ---
        if any(curr_state(1:2) > patch_size) || any(curr_state(1:2) < 0)
            boundary_hit = true;
            break;
        end
    end

    % =============================================
    % PLOTTING (Live visualizer updates)
    % =============================================
    s1 = curr_state(1) - xd(1);
    s2 = curr_state(2) - xd(2);
    s3 = curr_state(7) - xd(3);
    s3 = mod(s3 + pi, 2*pi) - pi;

    t_buf  = [t_buf(2:end),  mission_time];
    wR_buf = [wR_buf(2:end), u(1)];
    wL_buf = [wL_buf(2:end), u(2)];
    s1_buf = [s1_buf(2:end), s1];
    s2_buf = [s2_buf(2:end), s2];
    s3_buf = [s3_buf(2:end), s3];

    mu_r_act_buf = [mu_r_act_buf(2:end), mu_r_act_hold];
    mu_l_act_buf = [mu_l_act_buf(2:end), mu_l_act_hold];
    mu_r_est_buf = [mu_r_est_buf(2:end), mu_r_est_hold];
    mu_l_est_buf = [mu_l_est_buf(2:end), mu_l_est_hold];

    % Update 9-DOF estimation buffers
    pos_true_buf = [pos_true_buf(:, 2:end), curr_state(1:3)];
    pos_est_buf  = [pos_est_buf(:, 2:end),  pos_est];
    vel_true_buf = [vel_true_buf(:, 2:end), curr_state(4:6)];
    vel_est_buf  = [vel_est_buf(:, 2:end),  vel_est];
    att_true_buf = [att_true_buf(:, 2:end), [yaw_true; pitch_true; roll_true]];
    att_est_buf  = [att_est_buf(:, 2:end),  [yaw_est; pitch_est; roll_est]];

    win_start = max(0, mission_time - ekg_len*dt*steps_per_frame);
    win_end   = mission_time + 0.5;

    % Draw Control EKG
    set(hWR, 'XData', t_buf, 'YData', wR_buf);
    set(hWL, 'XData', t_buf, 'YData', wL_buf);
    xlim(ax_ekg, [win_start, win_end]);

    set(hS1, 'XData', t_buf, 'YData', s1_buf);
    set(hS2, 'XData', t_buf, 'YData', s2_buf);
    set(hS3, 'XData', t_buf, 'YData', s3_buf);
    xlim(ax_err, [win_start, win_end]);

    set(hMuRact, 'XData', t_buf, 'YData', mu_r_act_buf);
    set(hMuRest, 'XData', t_buf, 'YData', mu_r_est_buf);
    set(hMuLact, 'XData', t_buf, 'YData', mu_l_act_buf);
    set(hMuLest, 'XData', t_buf, 'YData', mu_l_est_buf);
    xlim(ax_mu, [win_start, win_end]);

    % Draw 9-DOF Live Estimation EKG
    for row = 1:3
        set(h_true(row, 1), 'XData', t_buf, 'YData', pos_true_buf(row, :));
        set(h_est(row, 1),  'XData', t_buf, 'YData', pos_est_buf(row, :));
        xlim(ax_handles(row, 1), [win_start, win_end]);
        
        set(h_true(row, 2), 'XData', t_buf, 'YData', vel_true_buf(row, :));
        set(h_est(row, 2),  'XData', t_buf, 'YData', vel_est_buf(row, :));
        xlim(ax_handles(row, 2), [win_start, win_end]);

        set(h_true(row, 3), 'XData', t_buf, 'YData', (att_true_buf(row, :)));
        set(h_est(row, 3),  'XData', t_buf, 'YData', (att_est_buf(row, :)));
        xlim(ax_handles(row, 3), [win_start, win_end]);
    end

    % Update 3D Environment
    p = curr_state(1:3);
    days_elapsed = mission_time / (24 * 3600);
    s_dir = LanderSunPointingVector(2831, days_elapsed);

    set(hSun, 'UData', s_dir(1)*sun_scale, 'VData', s_dir(2)*sun_scale, 'WData', s_dir(3)*sun_scale);
    set(hSunPt, 'XData', lander_pos(1)+s_dir(1)*sun_scale, 'YData', lander_pos(2)+s_dir(2)*sun_scale, 'ZData', lander_pos(3)+s_dir(3)*sun_scale);
    set(hRover, 'XData', p(1), 'YData', p(2), 'ZData', p(3));

    set(hRover_quiver, 'UData', p(1) - lander_pos(1), 'VData', p(2) - lander_pos(2), 'WData', p(3) - lander_pos(3));
    set(hRover_sun, ...
        'XData', p(1), 'YData', p(2), 'ZData', p(3),...
        'UData', s_dir(1) * sun_scale - (p(1) - lander_pos(1)), ...
        'VData', s_dir(2) * sun_scale - (p(2) - lander_pos(2)), ...
        'WData', s_dir(3)* sun_scale - (p(3) - lander_pos(3)));

    R_r_gravity_viz = R_true' * [0; 0; -1];
    R_r_gravity_viz = R_r_gravity_viz / norm(R_r_gravity_viz);
    R_r_sun_viz = R_true' * s_dir;
    R_r_sun_viz = R_r_sun_viz / norm(R_r_sun_viz);
    L_r_sun_viz = s_dir / norm(s_dir);

    rot_matrix = TRIAD(L_r_sun_viz, R_r_sun_viz, R_r_gravity_viz);
    L_attitude_rover = rot_matrix * [1; 0; 0] * scale;

    set(h_attitude, ...
        'XData', p(1), 'YData', p(2), 'ZData', p(3),...
        'UData', L_attitude_rover(1), ...
        'VData', L_attitude_rover(2), ...
        'WData', L_attitude_rover(3));

    f = R_true(:,1)*scale; l = R_true(:,2)*scale; u_vec = R_true(:,3)*scale;
    set(hFwd, 'XData', p(1), 'YData', p(2), 'ZData', p(3), 'UData', f(1), 'VData', f(2), 'WData', f(3));
    set(hLft, 'XData', p(1), 'YData', p(2), 'ZData', p(3), 'UData', l(1), 'VData', l(2), 'WData', l(3));
    set(hUp,  'XData', p(1), 'YData', p(2), 'ZData', p(3), 'UData', u_vec(1), 'VData', u_vec(2), 'WData', u_vec(3));

    drawnow limitrate;
end

fprintf('Sim complete: %.1f s mission time, %d steps\n', mission_time, i);
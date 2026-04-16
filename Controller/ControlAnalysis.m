clear; clc; close all;

%% 1. Physics Parameters

K = [0.1; 0.1; 2.0];       % SMC gain
eps_v = 0.01;              % sigmoid smoothing

r = 0.017; B = 0.2;        % rover dims

max_w = 105 * (2*pi)/60;   % motor restrictions (max rad/s)

f_phys = 1000;              % sim rate
f_ctrl = 100;               % ctrl rate
dt = 1/f_phys;              % time step for sim
decim = f_phys/f_ctrl;

noise_scales = [0.01;       % X vel process noise
                0.01;       % Y vel process noise
                0.05];      % omega process noise

%% 2. Setup Grids (M, F, N Control)

range_R = 0.6:0.02:0.98;    % mu_r range (now includes nominal 1.0)
range_L = 0.6:0.02:0.98;    % mu_l range (now includes nominal 1.0)
[MU_R, MU_L] = meshgrid(range_R, range_L);

mu_R_base = MU_R(:)';
mu_L_base = MU_L(:)';

% --- BATCHING CONTROLS ---
M = 64;                     % Number of distinct Reference Trajectories
F = length(mu_R_base);      % Number of Friction Scenarios (Grid size)
N = 100;                    % Number of Iterations per Trajectory
W = M * F * N;              % Total batched simulations to run concurrently

% Make the entire thing 2D. we repeat the same iteration N times, in a row,
% and then repeat each set 64 times to repeat the same test case for each
% trajectory.
mu_R_mega = repmat(repelem(mu_R_base, 1, N), 1, M);
mu_L_mega = repmat(repelem(mu_L_base, 1, N), 1, M);
%% 3. Generate Reference Trajectories (quarter + 4-way mirror)

% Major issue existed where trajectories are generated biasing a certain
% direction, so when the rover moves, the slipping wheel in that direction
% ends up pulling the rover off course more than if the slipping wheel was
% facing the opposite direction. This biases the plots, so we mirror the
% trajectory in all 4 directions to account for clockwise/counter-clockwise
% based biases.

% Spline control points
t_knots = 0:5:30;
Traj_dummy = Trajectory(f_phys, t_knots, 0, 0, 0.00, 0.055);
n_steps = length(Traj_dummy.t_master);

assert(mod(M,4) == 0, "Chosen # of trajectories must be divisible by 4!")
M_qtr = M / 4;  

% This is our desired state and state derivative that we stack as an M x
% n_steps array. Note: n_steps is basically the number of steps our physics
% is going to to step forward in time. 
t_X_all  = zeros(M, n_steps); 
t_Y_all  = zeros(M, n_steps); 
t_Th_all  = zeros(M, n_steps);

t_Xd_all = zeros(M, n_steps); 
t_Yd_all = zeros(M, n_steps); 
t_Thd_all = zeros(M, n_steps);

Traj_List = cell(M, 1);

fprintf('Generating %d Trajectories (%d unique)...\n', M, M_qtr);

for m = 1:M_qtr
    % Generate the trajectory and save it, but also save the reflections
    % next to each other
    Traj = Trajectory(f_phys, t_knots, 0, 0, 0.00, 0.055);
    Traj_List{m} = Traj;
    
    % 1. Original:  (X,  Y,  theta, Xd,  Yd,  theta_d)
    t_X_all(m,:)  = Traj.X;       
    t_Y_all(m,:)  = Traj.Y;
    t_Th_all(m,:) = Traj.Theta;
    
    t_Xd_all(m,:) = Traj.X_dot;   
    t_Yd_all(m,:) = Traj.Y_dot;
    t_Thd_all(m,:)= Traj.Theta_dot;

    % 2. Reflect across Y:  (X, -Y, -theta,      Xd, -Yd, -theta_d)
    j = M_qtr + m;
    t_X_all(j,:)  =  Traj.X;       
    t_Y_all(j,:)  = -Traj.Y;
    t_Th_all(j,:) = -Traj.Theta;
    
    t_Xd_all(j,:) =  Traj.X_dot;   
    t_Yd_all(j,:) = -Traj.Y_dot;
    t_Thd_all(j,:)= -Traj.Theta_dot;

    % 3. Reflect across X:  (-X,  Y,  π-theta,  -Xd,  Yd, -theta_d)
    j = 2*M_qtr + m;
    t_X_all(j,:)  = -Traj.X;       
    t_Y_all(j,:)  =  Traj.Y;
    t_Th_all(j,:) =  pi - Traj.Theta;
    
    t_Xd_all(j,:) = -Traj.X_dot;   
    t_Yd_all(j,:) =  Traj.Y_dot;
    t_Thd_all(j,:)= -Traj.Theta_dot;

    % 4. Reflect XY: (-X, -Y,  θ+theta,  -Xd, -Yd,  theta_d)
    j = 3*M_qtr + m;
    t_X_all(j,:)  = -Traj.X;       
    t_Y_all(j,:)  = -Traj.Y;
    t_Th_all(j,:) =  Traj.Theta + pi;
    
    t_Xd_all(j,:) = -Traj.X_dot;   
    t_Yd_all(j,:) = -Traj.Y_dot;
    t_Thd_all(j,:)=  Traj.Theta_dot;
end

%% 4. Compile the MEX file

% Do this in C. It has a tendency to be faster when programmed properly
fprintf('Compiling C++ MEX file...\n');
cfg = coder.config("mex");
cfg.IntegrityChecks = false;            % Dangerous
cfg.ResponsivenessChecks = false;       % Dangerous
cfg.ExtrinsicCalls = false;             % Don't talk to nobody
cfg.OptimizeReductions = true;          % Let the compiler take over
cfg.EnableOpenMP = true;                % Multi-processing!

% Right before the codegen / MEX call, cast everything to single precision
K = single(K); 
eps_v = single(eps_v); 
r = single(r); B = single(B);

max_w = single(max_w); 
dt = single(dt); 
noise_scales = single(noise_scales);
mu_R_mega = single(mu_R_mega); 
mu_L_mega = single(mu_L_mega);

t_X_all = single(t_X_all); 
t_Y_all = single(t_Y_all); 
t_Th_all = single(t_Th_all);

t_Xd_all = single(t_Xd_all); 
t_Yd_all = single(t_Yd_all); 
t_Thd_all = single(t_Thd_all);

% run the codegen loop
codegen smc_core_loop -args {K, eps_v, r, B, max_w, dt, decim, noise_scales, mu_R_mega, mu_L_mega, t_X_all, t_Y_all, t_Th_all, t_Xd_all, t_Yd_all, t_Thd_all, M, F, N, W, n_steps} -config cfg -report

%% 5. RUN THE COMPILED C++ LOOP

fprintf('Executing batched MEX simulation (Total runs: %d)...\n', W);

tic;
[mega_max_spatial, mega_max_theta, sum_x, sum_y, sum_x2, sum_y2] = ...
    smc_core_loop_mex(K, eps_v, r, B, max_w, dt, decim, noise_scales, ...
                      mu_R_mega, mu_L_mega, t_X_all, t_Y_all, t_Th_all, ...
                      t_Xd_all, t_Yd_all, t_Thd_all, M, F, N, W, n_steps);
fprintf('Simulation finished in %.2f seconds.\n', toc);

%% 6. Data Unpacking

% --- Heatmaps ---

% Get the spatial and theta data
spat_3D = reshape(mega_max_spatial, N, F, M);
th_3D = reshape(mega_max_theta, N, F, M);

% reduce the size of the data, and just save the mean errors
mean_per_scenario_spatial = reshape(mean(mean(spat_3D, 1), 3), size(MU_R));
mean_per_scenario_theta = reshape(mean(mean(th_3D, 1), 3), size(MU_R));

% Throw away the extra data otherwise we WILL get an out of memory error
clear spat_3D th_3D mega_max_spatial mega_max_theta;

% --- Envelopes: extract best-case first then free the big arrays ---
% sum_x is (M, F, n_steps), so lets get that idx 
% (corresponds to best mu_r, mu_l)
sx  = squeeze(sum_x(:, F, :));    % (M, n_steps)
sy  = squeeze(sum_y(:, F, :));
sx2 = squeeze(sum_x2(:, F, :));
sy2 = squeeze(sum_y2(:, F, :));

% Throw away literally everything else
clear sum_x sum_y sum_x2 sum_y2;

% calculate the sample means, sample variances
mean_x_nom = sx / N;
mean_y_nom = sy / N;
std_x_nom  = sqrt(max(0, sx2 / N - mean_x_nom.^2));
std_y_nom  = sqrt(max(0, sy2 / N - mean_y_nom.^2));

% Reduce memory overhead even further
clear sx sy sx2 sy2;

%% 8. Heatmaps

if numel(MU_R) > 3
    figure('Name', 'Worst-Case Heatmaps', 'Position', [100, 100, 1000, 450]); clf;

    subplot(1,2,1);
    contourf(MU_R, MU_L, mean_per_scenario_spatial * 100, 20, 'LineStyle', 'none');
    colorbar; hold on;
    plot(1.0, 1.0, 'w*', 'MarkerSize', 10, 'LineWidth', 2);
    title('Mean Spatial Error (cm)');
    xlabel('\mu_R (Actual)'); ylabel('\mu_L (Actual)'); axis square;

    subplot(1,2,2);
    contourf(MU_R, MU_L, mean_per_scenario_theta * 180 / pi, 20, 'LineStyle', 'none');
    colorbar; hold on;
    plot(1.0, 1.0, 'w*', 'MarkerSize', 10, 'LineWidth', 2);
    title('Mean Heading Error (deg)');
    xlabel('\mu_R (Actual)'); ylabel('\mu_L (Actual)'); axis square;

    sgtitle('Mean Across Paths');
else
    fprintf('Skipping Heatmap (friction grid too small).\n');
end

%% 9. Single Trajectory

figure('Name', 'SMC Tracking Verification', 'Position', [200, 200, 800, 600]);
clf; hold on; grid on; axis equal;

% select random trajectory
idx = randi([1, M]);

% plot it.
plot(t_X_all(idx, :), t_Y_all(idx, :), 'k--', 'LineWidth', 2, 'DisplayName', 'Reference Path');
plot(mean_x_nom(idx, :), mean_y_nom(idx, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Mean Path (\mu=1)');

% Normal vector to the reference for projecting the envelope width
norms = sqrt(t_Xd_all(idx,:).^2 + t_Yd_all(idx,:).^2) + 1e-6;
nx = -t_Yd_all(idx,:) ./ norms;
ny =  t_Xd_all(idx,:) ./ norms;
std_dist = sqrt(std_x_nom(idx,:).^2 + std_y_nom(idx,:).^2);

% 1-sigma 
sigmas = [1, 2, 3];
alphas = [0.4, 0.2, 0.1];
num_pts = length(nx);
step = 1;

% draw widest first so narrower layers paint on top
for s = length(sigmas):-1:1      
    b1x = mean_x_nom(idx,:) + sigmas(s) * std_dist .* nx;
    b1y = mean_y_nom(idx,:) + sigmas(s) * std_dist .* ny;
    b2x = mean_x_nom(idx,:) - sigmas(s) * std_dist .* nx;
    b2y = mean_y_nom(idx,:) - sigmas(s) * std_dist .* ny;

    ii = 1:step:(num_pts - step);
    px = [b1x(ii); b1x(ii+step); b2x(ii+step); b2x(ii)];
    py = [b1y(ii); b1y(ii+step); b2y(ii+step); b2y(ii)];

    if s == length(sigmas)
        patch(px, py, 'b', 'FaceAlpha', alphas(s), 'EdgeColor', 'none', ...
              'DisplayName', sprintf('%d\\sigma Envelope', sigmas(end)));
    else
        patch(px, py, 'b', 'FaceAlpha', alphas(s), 'EdgeColor', 'none', ...
              'HandleVisibility', 'off');
    end
end

% Start position
plot(t_X_all(idx, 1), t_Y_all(idx, 1), 'k^', 'MarkerSize', 8, 'MarkerFaceColor', 'k', ...
     'DisplayName', 'Start');
title(sprintf('SMC Tracking (Testcase %d, Nominal \\mu=1.0)', idx), ...
      'FontSize', 14, 'FontWeight', 'bold');

xlabel('X Position (m)'); ylabel('Y Position (m)');
legend('Location', 'best', 'FontSize', 12);
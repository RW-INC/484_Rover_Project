

state = [2; 0; 0; 0; 0; 0; 0; 0];
orientation = quaternion(1, 0, 0, 0);
u = [0; 0];

prior_estimate = state;
prior_P = diag([ones([1, 3]), 0.1 * ones([1, 3]), 0.01, 0.01]) * 0.1;
normaldist_pos = makedist('Normal', 'mu', 0, 'sigma', 0.03);
normaldist_rot = makedist('Normal', 'mu', 0, 'sigma', 0.15 * 2 * pi / 360);
normaldist_vel = makedist('Normal', 'mu', 0, 'sigma', 0.003);
normaldist_rho = makedist('Normal', 'mu', 0, 'sigma', 0.01);
normaldist_rhodot = makedist('Normal', 'mu', 0, 'sigma', 0.001);
dt = 0.1;
timesteps = 1:5000;
rw = 0.085;
B = 0.2;

act = [];
est = [];
err = [];

for i=timesteps
    const = 2.35;
    u_new = [const * sin(deg2rad(i) / (2 * pi)); const * sin(deg2rad(i) / (2 * pi))];
    
    du = u_new - u;
    u = u_new;
    eul = quat2eul(orientation, 'xyz');
    pitch = eul(2);
    yaw = eul(3);
    theta_dot = cos(pitch)^2 * (state(6) - tan(pitch) * (state(4) * cos(yaw) + state(5) * sin(yaw))) / sqrt(state(1)^2 + state(2)^2);
    
    state_dot = (rw / 2) * [cos(yaw) * (u(1) + u(2)); 
                 sin(yaw) * (u(1) + u(2)); 
                 sin(pitch) * (u(1) + u(2)); 
                 cos(yaw) * sum(du); 
                 sin(yaw) * sum(du); 
                 sin(pitch) * sum(du); 
                 rw * (u(1) - u(2)) / B; 
                 theta_dot];

    state = state + dt * state_dot;
    IMU = [state(1:3) + random(normaldist_pos, [3, 1]); 
           state(4:6) + random(normaldist_vel, [3, 1])];
    UWB = [norm(state(1:3)) + random(normaldist_rho, 1); 
        dot(state(1:3), state(4:6)) / norm(state(1:3)) + ...
        random(normaldist_rhodot, 1)];

    q_noise = quaternion([1, random(normaldist_rot, [1, 3]) / 2]);
    q_noise = q_noise / norm(q_noise);

    orientation_estimate = quatmultiply(orientation, q_noise);

    [state_hat, P] = translational_ekf(IMU, UWB, orientation_estimate, ...
        [u; du], prior_estimate, prior_P);

    prior_estimate = state_hat;
    prior_P = P;

    act = [act; state'];
    est = [est; state_hat'];
    err = [err; state_hat' - state'];
end

subplot(3, 2, 1)
hold on
plot(timesteps / 10, act(:, 1), 'b-', 'LineWidth', 2)
plot(timesteps / 10, est(:, 1), 'r--', 'LineWidth', 1.5)
title("X")

subplot(3, 2, 3)
hold on
plot(timesteps / 10, act(:, 2), 'b-', 'LineWidth', 2)
plot(timesteps / 10, est(:, 2), 'r--', 'LineWidth', 1.5)
title("Y")

subplot(3, 2, 5)
hold on
plot(timesteps / 10, act(:, 3), 'b-', 'LineWidth', 2)
plot(timesteps / 10, est(:, 3), 'r--', 'LineWidth', 1.5)
title("Z")

subplot(3, 2, 2)
hold on
plot(timesteps / 10, act(:, 4), 'b-', 'LineWidth', 2)
plot(timesteps / 10, est(:, 4), 'r--', 'LineWidth', 1.5)
title("X Velo")

subplot(3, 2, 4)
hold on
plot(timesteps / 10, act(:, 5), 'b-', 'LineWidth', 2)
plot(timesteps / 10, est(:, 5), 'r--', 'LineWidth', 1.5)
title("Y Velo")

subplot(3, 2, 6)
hold on
plot(timesteps / 10, act(:, 6), 'b-', 'LineWidth', 2)
plot(timesteps / 10, est(:, 6), 'r--', 'LineWidth', 1.5)
title("Z Velo")

figure;
plot(timesteps, err)

function [state_hat, P] = translational_ekf(IMU, UWB, orientation_estimate, u, prior_estimate, prior_P)
    %measurement / control unpacking
    dt = 0.1;
    x_nminus1 = prior_estimate(1);
    y_nminus1 = prior_estimate(2);
    z_nminus1 = prior_estimate(3);
    x_dot_nminus1 = prior_estimate(4);
    y_dot_nminus1 = prior_estimate(5);
    z_dot_nminus1 = prior_estimate(6);
    rho_nminus1 = norm(prior_estimate(1:3));

    wr = u(1);
    wl = u(2);
    dwr = u(3);
    dwl = u(4);
    du = u(3:4);
    v = wr + wl;
    dv = dwr + dwl;
    
    eul = quat2eul(orientation_estimate, 'xyz');
    %roll = orientation_estimate(1);
    pitch = eul(2);
    yaw = eul(3);

    rw = 0.17/2; %wheel radius, m
    B = 0.2; %base width, m

    %process noise matrix
    W = diag([0.001; 0.001; 0.001; 0.0001; 0.0001; 0.0001; 0.0003; 0.0003]);

    % MEASUREMENT NOISE TODO
    R = diag([0.03, 0.03, 0.03, 0.003, 0.003, 0.003, 0.01, 0.001]); 
    %R = diag(ones(8) * 0.01);
    
    %predict step
    %state propagation
    planar = sqrt(x_nminus1^2 + y_nminus1^2);
    theta_dot = cos(pitch)^2 * (z_dot_nminus1 - tan(pitch) * (x_dot_nminus1 * cos(yaw) + y_dot_nminus1 * sin(yaw))) / planar;
    state_dot_hat = (rw / 2) * [cos(yaw) * (u(1) + u(2)); 
                 sin(yaw) * (u(1) + u(2)); 
                 sin(pitch) * (u(1) + u(2)); 
                 cos(yaw) * sum(du); 
                 sin(yaw) * sum(du); 
                 sin(pitch) * sum(du); 
                 rw * (u(1) - u(2)) / B; 
                 theta_dot];

    
    %rw * [cos(yaw) * (wr + wl) / 2; sin(yaw) * (wr + wl) / 2; (wr - wl) / B];
    state_hat_pred = prior_estimate + state_dot_hat * dt;
    
    jacobian = [0, 0, 0, 0, 0, 0, -rw * sin(yaw) * (wr + wl) / 2, 0; 
                0, 0, 0, 0, 0, 0, rw * cos(yaw) * (wr + wl) / 2, 0; 
                0, 0, 0, 0, 0, 0, 0, rw * cos(pitch) * (wr + wl) / 2; 
                0, 0, 0, 0, 0, 0, -rw * sin(yaw) * (dwr + dwl) / 2, 0;
                0, 0, 0, 0, 0, 0, rw * cos(yaw) * (dwr + dwl) / 2, 0;
                0, 0, 0, 0, 0, 0, 0, rw * cos(pitch) * (wr + wl) / 2;
                0, 0, 0, 0, 0, 0, 0, 0; 
                -theta_dot * x_nminus1 * x_dot_nminus1 / (2 * planar^2), ...
                -theta_dot * y_nminus1 * y_dot_nminus1 / (2 * planar^2), ...
                0, ...
                -sin(pitch) * cos(pitch) * cos(yaw) / planar, ...
                -sin(pitch) * cos(pitch) * sin(yaw) / planar, ...
                cos(pitch)^2 / planar, ...
                (x_dot_nminus1 * sin(yaw) + y_dot_nminus1 * cos(yaw)) * sin(pitch) * cos(pitch) / planar, ...
                (-z_dot_nminus1 * sin(2 * pitch) - cos(2 * pitch) * (x_dot_nminus1 * cos(yaw) + y_dot_nminus1 * sin(yaw))) / planar];
    
    %covariance propagation
    P_dot = jacobian * prior_P + prior_P * jacobian' + W;
    P_pred = prior_P + P_dot * dt;


    x_n = state_hat_pred(1);
    y_n = state_hat_pred(2);
    z_n = state_hat_pred(3);
    x_dot_n = state_hat_pred(4);
    y_dot_n = state_hat_pred(5);
    z_dot_n = state_hat_pred(6);
    rho_n = norm(state_hat_pred(1:3));


    %update step
    alpha = dot(state_hat_pred(1:3), state_hat_pred(4:6)); 
    lambda = atan2(y_dot_n, x_dot_n);
    c2l = cos(lambda)^2;
    temp = [1, 0, 0, 0, 0, 0, 0, 0; 
            0, 1, 0, 0, 0, 0, 0, 0; 
            0, 0, 1, 0, 0, 0, 0, 0;
            0, 0, 0, 1, 0, 0, 0, 0;
            0, 0, 0, 0, 1, 0, 0, 0;
            0, 0, 0, 0, 0, 1, 0, 0];
    H = [temp; 
        x_n / rho_n, y_n / rho_n, z_n / rho_n, zeros(1, 5); 
        (x_dot_n * rho_n - alpha * x_n / rho_n) / rho_n^2, ...
        (y_dot_n * rho_n - alpha * y_n / rho_n) / rho_n^2, ...
        (z_dot_n * rho_n - alpha * z_n / rho_n) / rho_n^2, ...
        x_n / rho_n, y_n / rho_n, z_n / rho_n, 0, 0]; 
        %0, 0, 0, c2l * -y_dot_n / x_dot_n^2, c2l / x_dot_n, 0];
    K = P_pred * H' * pinv(H * P_pred * H' + R);
    rho_pred = norm(state_hat_pred(1:3));
    alpha_pred = dot(state_hat_pred(1:3), state_hat_pred(4:6));
    state_hat = state_hat_pred + K * ([IMU; UWB] - [state_hat_pred(1:6); ...
        rho_pred; alpha_pred / rho_pred]);
    P = (eye(8) - K * H) * P_pred;

end

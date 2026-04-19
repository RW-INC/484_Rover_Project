function [state_hat, P] = translational_ekf_fixed(IMU, UWB, orientation_estimate, u, prior_estimate, prior_P, rot_P, dt)
    %measurement / control unpacking

    wr = u(1);
    wl = u(2);
    dwr = u(3);
    dwl = u(4);
    v = wr + wl;
    dv = dwr + dwl;
    dv = dv / dt;
    
    eul = orientation_estimate;
    %roll = orientation_estimate(1);
    pitch = eul(2);
    yaw = eul(3);

    rw = 0.17/2; %wheel radius, m
    % B = 0.2; %base width, m

    %process noise matrix
    W = diag([0.001; 0.001; 0.001; 0.0001; 0.0001; 0.0001]);

    % MEASUREMENT NOISE TODO
    R = diag([0.03^2, 0.03^2, 0.03^2, 0.003^2, 0.003^2, 0.003^2, 0.01^2, 0.001^2]); 
    %R = diag(ones(8) * 0.01);
    
    %predict step
    %state propagation
    state_dot_hat = [cos(yaw) * v; 
                    sin(yaw) * v; 
                    sin(pitch) * v; 
                    cos(yaw) * dv; 
                    sin(yaw) * dv; 
                    sin(pitch) * dv] * rw / 2;

    
    %rw * [cos(yaw) * (wr + wl) / 2; sin(yaw) * (wr + wl) / 2; (wr - wl) / B];
    state_hat_pred = prior_estimate + state_dot_hat * dt;
    
    %covariance propagation
    P_dot = W;
    P_pred = prior_P + P_dot * dt;

    x_n = state_hat_pred(1);
    y_n = state_hat_pred(2);
    z_n = state_hat_pred(3);
    x_dot_n = state_hat_pred(4);
    y_dot_n = state_hat_pred(5);
    z_dot_n = state_hat_pred(6);
    rho_n = norm(state_hat_pred(1:3));

    %update step
    alpha = dot(prior_estimate(1:3), prior_estimate(4:6)); 
    
    % yaw_angle_dynamics = atan2(y_n, x_n);
    % theta_angle_dynamics = atan2(z_n, sqrt(x_n ^ 2 + y_n^2));
    % 
    % c2y = cos(yaw_angle_dynamics)^2;
    % c2t = cos(theta_angle_dynamics)^2;

    H = [eye(6); 
        x_n / rho_n, y_n / rho_n, z_n / rho_n, zeros(1, 3); 
        (x_dot_n * rho_n - alpha * x_n / rho_n) / rho_n^2, ...
        (y_dot_n * rho_n - alpha * y_n / rho_n) / rho_n^2, ...
        (z_dot_n * rho_n - alpha * z_n / rho_n) / rho_n^2, ...
        x_n / rho_n, y_n / rho_n, z_n / rho_n;
        % -c2y * y_n/(x_n)^2, c2y / x_n, 0, 0, 0, 0; 
        % -0.25 * sin(2 * theta_angle_dynamics) * (x_n * x_dot_n) / (x_n ^ 2 + y_n^2), ...
        % -0.25 * sin(2 * theta_angle_dynamics) * (y_n * y_dot_n) / (x_n^2 + y_n^2), ...
        % -c2t * (1 / sqrt(x_n^2 + y_n ^ 2)), 0, 0, 0
        ];

    K = P_pred * H' * pinv(H * P_pred * H' + R);
    rho_pred = norm(state_hat_pred(1:3));
    alpha_pred = dot(state_hat_pred(1:3), state_hat_pred(4:6));
    state_hat = state_hat_pred + K * ([IMU; UWB] - [state_hat_pred; rho_pred; alpha_pred / rho_pred]);
    P = (eye(6) - K * H) * P_pred;
end
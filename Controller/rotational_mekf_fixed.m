function [state, P] = rotational_mekf_fixed(q_est_triad, w_gyro, prior_state, prior_P, u, dt, sigma_gyro)
% ROTATIONAL_MEKF_FIXED  Multiplicative EKF for attitude estimation.
%   q_est_triad  - quaternion from TRIAD (MATLAB quaternion object)
%   w_gyro       - gyroscope measurement [3x1] (rad/s, body frame)
%   prior_state  - [qw, qx, qy, qz, bx, by, bz] previous estimate
%   prior_P      - [6x6] previous error covariance
%   u            - [wR; wL] wheel speeds
%   dt           - timestep (s)
%
%   Returns:
%   state - [qw, qx, qy, qz, bx, by, bz] updated estimate
%   P     - [6x6] updated error covariance

    prior_q = quaternion(prior_state(1), prior_state(2), prior_state(3), prior_state(4));
    prior_b = prior_state(5:7)';
    rw = 0.085;
    B = 0.2;

    % --- Predict: quaternion propagation ---
    w_corrected = w_gyro - prior_b + [0; 0; rw * (u(1) - u(2)) / B];
    q_dot = 0.5 * quatmultiply(prior_q, quaternion(0, w_corrected(1), w_corrected(2), w_corrected(3)));
    q_pred = prior_q + q_dot * dt;
    q_pred = q_pred / (norm(q_pred) + 1e-12);
    b_pred = prior_b;

    % --- Predict: error covariance ---
    w_x = [0, -w_corrected(3), w_corrected(2); 
           w_corrected(3), 0, -w_corrected(1); 
           -w_corrected(2), w_corrected(1), 0];
    F = [-w_x, -eye(3); zeros(3), zeros(3)];

    Q_theta = (sigma_gyro)^2 * dt * eye(3);
    Q_b = (6.5e-5)^2 * dt * eye(3); % 0.04 mg on the moon? 
    P = prior_P + (F * prior_P + prior_P * F' + blkdiag(Q_theta, Q_b)) * dt;

    % --- Update: measurement from TRIAD ---
    q_err = conj(q_pred) * q_est_triad;
    [~, ey, ec, ed] = parts(q_err);
    dtheta = 2 * [ey; ec; ed];

    H = [eye(3), zeros(3)];
    R = 8.7e-3 * eye(3);
    S = H * P * H' + R;
    K = P * H' * pinv(S);
    dx = K * dtheta;

    dtheta_corr = dx(1:3);
    db = dx(4:6);

    % Small angle quaternion correction
    angle = norm(dtheta_corr);
    if angle < 1e-8
        dq_vec = [1, 0.5*dtheta_corr'];
    else
        ax = dtheta_corr / angle;
        dq_vec = [cos(angle/2), sin(angle/2)*ax'];
    end
    dq = quaternion(dq_vec(1), dq_vec(2), dq_vec(3), dq_vec(4));

    q = quatmultiply(q_pred, dq);
    q = q / (norm(q) + 1e-12);
    b = b_pred + db;

    % Joseph form for numerical stability
    P = (eye(6) - K * H) * P * (eye(6) - K * H)' + K * R * K';
    
    [qw, qx, qy, qz] = parts(q);
    
    if qw < 0
        qw = -qw;
        qx = -qx; 
        qy = -qy; 
        qz = -qz;
    end
    state = [qw, qx, qy, qz, b(1), b(2), b(3)];
end
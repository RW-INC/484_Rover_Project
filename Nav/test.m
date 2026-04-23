    % =========================================================================
    % CONTINUOUS-DISCRETE MEKF (FULL INTEGRATED SCRIPT)
    % =========================================================================
    clear; clc;
    
    % --- 1. INITIALIZATION ---
    q_init = [1, 0, 0, 0]; 
    b_init = [0.001, 0.001, 0.001];
    prior_state = [q_init, b_init];
    w_gyro = [1; 1; 1] * pi / 180; 
    prior_P = blkdiag(0.01 * eye(3), 0.01 * eye(3)) * pi / 180;
    dt = 0.1;
    
    % Keep noise realistic for testing
    normaldist = makedist('Normal', 'mu', 0, 'sigma', 0.15 * pi / 180); 
    
    % True simulated state
    q_true = quaternion(q_init);
    b_true = b_init';
    
    angles = [];
    true_angles = []; 
    time_steps = 1:5000;

    for i = time_steps
        % ==========================
        % 1. SIMULATE TRUE PHYSICS (Continuous)
        % ==========================
        [w, x, y, z] = parts(q_true);
        q_true_arr = [w; x; y; z];
        
        % Integrate truth kinematics using ode45
        [~, q_true_sim] = ode45(@(t, y) true_dynamics(t, y, w_gyro), [0 dt], q_true_arr);
        q_true_arr = q_true_sim(end, :)';
        q_true_arr = q_true_arr / norm(q_true_arr); 
        q_true = quaternion(q_true_arr');
        
        % Track the true Euler angles
        true_angles = [true_angles; quat2eul(q_true, 'xyz')];
        
        % Simulate noisy TRIAD measurement (Discrete)
        noise_vec = random(normaldist, [1, 3]);
        q_noise = quaternion([1, noise_vec / 2]); 
        q_noise = q_noise / norm(q_noise);
        q_est_triad_meas = quatmultiply(q_true, q_noise); 
        
        % Simulate noisy gyro measurement (Discrete)
        w_gyro_meas = w_gyro + b_true * dt * i / 3600 + random(normaldist, [3, 1]);
        
        u = [0, 0];
        
        % ==========================
        % 2. RUN THE MEKF
        % ==========================
        [state, P] = rotational_mekf_cd(q_est_triad_meas, w_gyro_meas, prior_state, prior_P, u, dt);
        
        prior_state = state;
        prior_P = P;
        
        angles = [angles; quat2eul(quaternion(state(1:4)), 'xyz')];
    end
    
    % ==========================
    % PLOTTING TRUTH VS ESTIMATE
    % ==========================
    time = (time_steps) * dt;
    figure('Name', 'MEKF Performance', 'Position', [100, 100, 800, 600]);
    sgtitle('MEKF Attitude Estimate vs. True Dynamics');
    
    % ROLL
    subplot(3, 1, 1);
    plot(time, true_angles(:, 3), 'k-', 'LineWidth', 2); hold on;
    plot(time, angles(:, 3), 'r--', 'LineWidth', 1.5);
    title('Roll'); ylabel('Radians'); legend('Truth', 'MEKF Estimate', 'Location', 'best'); grid on;
    
    % PITCH
    subplot(3, 1, 2);
    plot(time, true_angles(:, 2), 'k-', 'LineWidth', 2); hold on;
    plot(time, angles(:, 2), 'g--', 'LineWidth', 1.5);
    title('Pitch'); ylabel('Radians'); legend('Truth', 'MEKF Estimate', 'Location', 'best'); grid on;
    
    % YAW
    subplot(3, 1, 3);
    plot(time, true_angles(:, 1), 'k-', 'LineWidth', 2); hold on;
    plot(time, angles(:, 1), 'b--', 'LineWidth', 1.5);
    title('Yaw'); xlabel('Time (seconds)'); ylabel('Radians'); legend('Truth', 'MEKF Estimate', 'Location', 'best'); grid on;
    
    % =========================================================================
    % MEKF FUNCTION (Continuous Predict, Discrete Update)
    % =========================================================================
    function [state, P] = rotational_mekf_cd(q_meas, w_meas, prior_state, prior_P, u, dt)
        prior_q = prior_state(1:4)'; 
        prior_b = prior_state(5:7)';
    
        % Process Noise Spectral Density Matrix W
        W = blkdiag((4.36e-8)^2 * eye(3), (4e-8)^2 * eye(3));
    
        % --- PREDICT STEP (Continuous via ODE45) ---
        y_init = [prior_q; prior_b; prior_P(:)];
        [~, y_pred_all] = ode45(@(t, y) mekf_dynamics(t, y, w_meas, u, W), [0 dt], y_init);
        y_pred = y_pred_all(end, :)'; 
        
        q_pred_arr = y_pred(1:4);
        q_pred_arr = q_pred_arr / norm(q_pred_arr); 
        q_pred = quaternion(q_pred_arr');
        b_pred = y_pred(5:7);
        P_pred = reshape(y_pred(8:43), 6, 6); 
        
        % --- UPDATE STEP (Discrete) ---
        q_err = quatmultiply(conj(q_pred), q_meas);
        [w, x, y, z] = parts(q_err);
        dtheta = 2 * [x; y; z]; 
        
        H = [eye(3), zeros(3, 3)];
        R = 8.7e-5 * eye(3);
        
        S = H * P_pred * H' + R;
        K = P_pred * H' / S; 
        dx = K * dtheta;
        
        % Rebuild Quaternion (Small Angle Safeguard)
        angle = norm(dx(1:3));
        % if angle < 1e-8
        dq = quaternion([1, 0.5 * dx(1:3)']);  
        % else
            % dq = quaternion(axang2quat([dx(1:3)' / angle, angle]));
        % end
        
        q = quatmultiply(q_pred, dq);
        q = q / norm(q); 
        b = b_pred + dx(4:6);
        
        % Pure Joseph Form Covariance Update
        I_KH = eye(6) - K * H;
        P = I_KH * P_pred * I_KH' + K * R * K';
        
        [w, x, y, z] = parts(q);
        state = [w, x, y, z, b(1), b(2), b(3)];
    end
    
    % =========================================================================
    % DIFFERENTIAL EQUATIONS FOR ODE45
    % =========================================================================
    % 1. MEKF Filter Prediction Dynamics
    function dydt = mekf_dynamics(~, y, w_meas, u, W)
        q = y(1:4);
        b = y(5:7);
        P = reshape(y(8:43), 6, 6);
        
        w_corrected = w_meas - b + [0; 0; 0.085 * (u(1) - u(2)) / 0.2];
        
        w_x = [0, -w_corrected(3), w_corrected(2); 
               w_corrected(3), 0, -w_corrected(1); 
               -w_corrected(2), w_corrected(1), 0];
        F = [-w_x, -eye(3); zeros(3, 6)];
        
        % Process Noise Mapping Matrix G
        G = blkdiag(-eye(3), eye(3));
        
        % Continuous Riccati (GWG^T formulation)
        P_dot = F * P + P * F' + G * W * G';
        
        % Pack derivative vector
        dydt = [q_kinematics(q, w_corrected); zeros(3, 1); P_dot(:)];
    end
    
    % 2. True Kinematics (For Simulation Engine)
    function dydt = true_dynamics(~, y, w_gyro)
        q = y(1:4);
        dydt = q_kinematics(q, w_gyro);
    end
    
    % 3. Matrix-Based Quaternion Kinematics
    function q_dot = q_kinematics(q, w)
        % 4x4 Skew-symmetric matrix
        Omega = [0,    -w(1), -w(2), -w(3); 
                 w(1),  0,     w(3), -w(2); 
                 w(2), -w(3),  0,     w(1); 
                 w(3),  w(2), -w(1),  0];
        q_dot = 0.5 * Omega * q;
    end
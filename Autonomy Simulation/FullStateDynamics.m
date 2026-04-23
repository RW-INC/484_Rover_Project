function dstate = FullStateDynamics(state, u, xmin, ymin, dx, dy, Z_map, geom, dt, ~)
%#codegen    
    yaw = state(9);
    pitch = state(8);
    w_r = u(1);  w_l = u(2);
    dw_r = u(3); dw_l = u(4);
    r = geom.r; B = geom.B; L = geom.L;
    
    % set mur, mul dependent on the dynamics
    % mu_r = 0.8 + 0.2 * exp(-abs(w_r));
    % mu_l = 0.8 + 0.2 * exp(-abs(w_l));

    mu_r = 1.0;
    mu_l = 1.0;
    % --- diff-drive kinematics ---
    v    = (r/2) * (mu_r * w_r + mu_l * w_l);
    % Velocities
    vx = v * cos(yaw) * cos(pitch);
    vy = v * sin(yaw) * cos(pitch);
    vz = v * sin(pitch);
    
    % Accelerations
    dv = (r/2) * (mu_r * dw_r + mu_l * dw_l)/(dt);
    % --- wheel positions & terrain query ---
    body_wheels = [ L/2  L/2 -L/2 -L/2;
                   -B/2  B/2 -B/2  B/2];
    
    % --- attitude rates (spatial perturbation) ---
    eps_p = 1e-6;
    [roll_px,   pitch_px]   = att(state(1)+eps_p, state(2), yaw);
    [roll_mx,   pitch_mx]   = att(state(1)-eps_p, state(2), yaw);
    
    [roll_py,   pitch_py]   = att(state(1), state(2)+eps_p, yaw);
    [roll_my,   pitch_my]   = att(state(1), state(2)-eps_p, yaw);
    
    [roll_pp,   pitch_pp]   = att(state(1), state(2), yaw+eps_p);
    [roll_mp,   pitch_mp]   = att(state(1), state(2), yaw-eps_p);
    
    dyaw = (r/B) * (mu_r * w_r - mu_l * w_l);
    droll  = (roll_px  - roll_mx) /(2*eps_p) * vx ...
           + (roll_py  - roll_my) /(2*eps_p) * vy ...
           + (roll_pp  - roll_mp) /(2*eps_p) * dyaw;
    dpitch = (pitch_px - pitch_mx)/(2*eps_p) * vx ...
           + (pitch_py - pitch_my)/(2*eps_p) * vy ...
           + (pitch_pp - pitch_mp)/(2*eps_p) * dyaw;
    % --- accelerations (u constant within step) ---
    ax = cos(yaw)*cos(pitch)*dv - v*cos(pitch)*sin(yaw)*dyaw - v*cos(yaw)*sin(pitch)*dpitch;
    ay = sin(yaw)*cos(pitch)*dv + v*cos(pitch)*cos(yaw)*dyaw - v*sin(yaw)*sin(pitch)*dpitch;
    az = sin(pitch)*dv + v*cos(pitch)*dpitch;
    % --- pack ---
    dstate = [vx; vy; vz; ax; ay; az; droll; dpitch; dyaw];
    % --- nested attitude helper ---
    
    function [rq, pq] = att(xq, yq, yq_yaw)
        Rq = [cos(yq_yaw) -sin(yq_yaw); sin(yq_yaw) cos(yq_yaw)];
        wq = Rq * body_wheels + [xq; yq];
        
        % zq = interp2(X_map, Y_map, Z_map, wq(1,:), wq(2,:), 'linear');
        zq = zeros(1, 4);
        for i = 1:4
            zq(i) = fast_triangle_interp(Z_map, xmin, ymin, dx, dy, wq(1,i), wq(2,i));
        end

        Aq = [wq(1,:)', wq(2,:)', ones(4,1)];
        % Replace abcq = Aq \ zq'; with this:
        At = Aq';
        abcq = (At * Aq) \ (At * zq');

        ab =  cos(yq_yaw)*abcq(1) + sin(yq_yaw)*abcq(2);
        bb = -sin(yq_yaw)*abcq(1) + cos(yq_yaw)*abcq(2);
        
        pq = atan(ab);
        rq = atan(bb);
    end

    function z = fast_triangle_interp(Z_map, xmin, ymin, dx, dy, x, y)
        x_rel = x - xmin;
        y_rel = y - ymin;
        
        idx_x_back = floor(x_rel / dx) + 1;
        idx_y_back = floor(y_rel / dy) + 1;
        idx_x_forward = ceil(x_rel / dx) + 1;
        idx_y_forward = ceil(y_rel / dy) + 1;
        
        % no integer bullshit
        if idx_x_forward == idx_x_back; idx_x_forward = idx_x_back + 1; end
        if idx_y_forward == idx_y_back; idx_y_forward = idx_y_back + 1; end
        
        z_bb = Z_map(idx_y_back, idx_x_back);       % Back-Back
        z_fb = Z_map(idx_y_back, idx_x_forward);    % Forward-Back 
        z_bf = Z_map(idx_y_forward, idx_x_back);    % Back-Forward
        z_ff = Z_map(idx_y_forward, idx_x_forward); % Forward-Forward
        
        % x,y values for these tings
        x_back = xmin + (idx_x_back - 1) * dx;
        y_back = ymin + (idx_y_back - 1) * dy;
        x_forward = xmin + (idx_x_forward - 1) * dx;
        y_forward = ymin + (idx_y_forward - 1) * dy;
        
        % Check if past the diagonal
        is_upper = ((x - x_back)/dx + (y - y_back)/dy) > 1.0; 
        
        % Calculate both planes (z = ax + by + c).
        % a, b i sjust using basic slope formula
        z_lower = ((z_fb - z_bb)/dx) * (x - x_back) + ...
                  ((z_bf - z_bb)/dy) * (y - y_back) + z_bb;
                  
        z_upper = ((z_ff - z_bf)/dx) * (x - x_forward) + ...
                  ((z_ff - z_fb)/dy) * (y - y_forward) + z_ff;
        
        % ternary expression.
        z = (~is_upper) * z_lower + (is_upper) * z_upper;
    end
end

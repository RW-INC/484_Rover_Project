function dstate = rover_dynamics(state, u, terrain, geom)
    yaw = state(7);
    pitch = state(8);
    w_r = u(1);  w_l = u(2);
    r = geom.r;  B = geom.B;  L = geom.L;

    % set mur, mul dependent on the dynamics
    mu_r = 0.75 + (0.23) * exp(-w_r);
    mu_l = 0.75 + (0.23) * exp(-w_l);

    % --- diff-drive kinematics ---
    v    = (r/2) * (mu_r * w_r + mu_l * w_l);
    dyaw = (r/B) * (mu_r * w_r - mu_l * w_l);
    vx   = v * cos(yaw);
    vy   = v * sin(yaw);

    % --- wheel positions & terrain query ---
    body_wheels = [ L/2  L/2 -L/2 -L/2;
                   -B/2  B/2 -B/2  B/2];
    R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
    ww = R * body_wheels + [state(1); state(2)];
    zw = interp2(terrain.X_map, terrain.Y_map, terrain.Z_map, ...
                 ww(1,:), ww(2,:), 'linear');
    
    % --- plane fit ---
    A = [ww(1,:)', ww(2,:)', ones(4,1)];
    abc = A \ zw';
    a = abc(1); b = abc(2);
    % pause;
    % --- z rate (chain rule, exact) ---
    vz = a*vx + b*vy;

    % --- attitude rates (spatial perturbation) ---
    eps_p = 1e-4;

    [roll_px,   pitch_px]   = att(state(1)+eps_p, state(2), yaw);
    [roll_mx,   pitch_mx]   = att(state(1)-eps_p, state(2), yaw);
    
    [roll_py,   pitch_py]   = att(state(1), state(2)+eps_p, yaw);
    [roll_my,   pitch_my]   = att(state(1), state(2)-eps_p, yaw);
    
    [roll_pp,   pitch_pp]   = att(state(1), state(2), yaw+eps_p);
    [roll_mp,   pitch_mp]   = att(state(1), state(2), yaw-eps_p);

    droll  = (roll_px  - roll_mx) /(2*eps_p) * vx ...
           + (roll_py  - roll_my) /(2*eps_p) * vy ...
           + (roll_pp  - roll_mp) /(2*eps_p) * dyaw;

    dpitch = (pitch_px - pitch_mx)/(2*eps_p) * vx ...
           + (pitch_py - pitch_my)/(2*eps_p) * vy ...
           + (pitch_pp - pitch_mp)/(2*eps_p) * dyaw;

    % --- accelerations (u constant within step) ---
    ax = -v * sin(yaw) * dyaw + v * cos(pitch) * dpitch;
    ay =  v * cos(yaw) * dyaw;
    az =  a * ax + b * ay + v * sin(pitch) * dpitch;

    % --- pack ---
    dstate = [vx; vy; vz; ax; ay; az; dyaw; dpitch; droll];

    % --- nested attitude helper ---
    function [rq, pq] = att(xq, yq, yq_yaw)
        Rq = [cos(yq_yaw) -sin(yq_yaw); sin(yq_yaw) cos(yq_yaw)];
        wq = Rq * body_wheels + [xq; yq];
        zq = interp2(terrain.X_map, terrain.Y_map, terrain.Z_map, ...
                     wq(1,:), wq(2,:), 'linear');
        Aq = [wq(1,:)', wq(2,:)', ones(4,1)];
        abcq = Aq \ zq';
        ab =  cos(yq_yaw)*abcq(1) + sin(yq_yaw)*abcq(2);
        bb = -sin(yq_yaw)*abcq(1) + cos(yq_yaw)*abcq(2);
        pq = atan(ab);
        rq = atan(bb);
    end
end
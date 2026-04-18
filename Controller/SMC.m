function u = SMC(x, xd, u, geom, max_w)
% SMC  Sliding-mode controller for differential-drive rover.
%   x   = [x; y; theta]          current state
%   xd  = [x; y; theta; xd; yd; thd]  reference state + velocities
%   u   = [wR; wL]               previous wheel speeds (modified in-place)
%   geom.r, geom.B               wheel radius, track width
%   max_w                         wheel speed saturation (rad/s)
    K = [0.02; 0.02; 0.1];
    eps_v = 2;

    r = geom.r;
    B = geom.B;
    inv_r    = 1 / r;
    r_over_2 = r / 2;
    r_over_B = r / B;
    half_B   = B / 2;

    K_1 = K(1); K_2 = K(2); K_3 = K(3);
    inv_2_epsv = 1 / (2 * eps_v);

    c_th = cos(x(3));
    s_th = sin(x(3));

    % Sliding variables
    s_1 = x(1) - xd(1);
    s_2 = x(2) - xd(2);
    s_3 = x(3) - xd(3);
    s_3 = mod(s_3 + pi, 2*pi) - pi;   % wrap to [-pi, pi]

    % Current v, omega from previous command
    v_nom = r_over_2 * (u(1) + u(2));
    w_nom = r_over_B * (u(1) - u(2));

    % Reaching law
    L_1 = -K_1 * tanh(s_1 * inv_2_epsv) - v_nom * c_th + xd(4);
    L_2 = -K_2 * tanh(s_2 * inv_2_epsv) - v_nom * s_th + xd(5);
    L_3 = -K_3 * tanh(s_3 * inv_2_epsv) - w_nom       + xd(6);

    % Project onto body frame and compute wheel increments
    L_proj = L_1 * c_th + L_2 * s_th;

    u_prev = u;
    u(1) = max(min(u(1) + (L_proj + L_3 * half_B) * inv_r,  max_w), -max_w);
    u(2) = max(min(u(2) + (L_proj - L_3 * half_B) * inv_r,  max_w), -max_w);
    u(3) = u(1) - u_prev(1);
    u(4) = u(2) - u_prev(2);
    
end
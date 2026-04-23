function [calc_J, calc_H] = init_dynamics(dynamics_function, arg_types)
    %% Symbolic Jacobian Setup
    syms theta_sym w_r_sym w_l_sym mu_R_sym mu_L_sym B_track_sym r_w_sym real
    
    v_sym = (r_w_sym * mu_R_sym / 2) * w_r_sym + (r_w_sym * mu_L_sym / 2) * w_l_sym;
    omega_sym = (r_w_sym * mu_R_sym / B_track_sym) * w_r_sym - (r_w_sym * mu_L_sym / B_track_sym) * w_l_sym;
    H_sym = [v_sym * cos(theta_sym); v_sym * sin(theta_sym); omega_sym];
    
    J_u_sym = jacobian(H_sym, [w_r_sym, w_l_sym]);
    
    calc_J = matlabFunction(J_u_sym, 'Vars', {theta_sym, w_r_sym, w_l_sym, mu_R_sym, mu_L_sym, B_track_sym, r_w_sym});
    calc_H = matlabFunction(H_sym, 'Vars', {theta_sym, w_r_sym, w_l_sym, mu_R_sym, mu_L_sym, B_track_sym, r_w_sym});

    f = func2str(dynamics_function);
    %codegen f -args {coder.typeof(0, [10 1]), coder.typeof(0, [4, 1]), coder.typeof(0), coder.typeof(0), coder.typeof(0), coder.typeof(0), coder.typeof(zeros(resolution, resolution)), coder.typeof(geom), coder.typeof(0), []} 
    codegen('-config', coder.config('mex'), f, '-args', arg_types);
end
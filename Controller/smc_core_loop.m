% This function is lowkey black magic and took me like a solid 6 to 7 hours
% to code up with the help of Gemini. And it took a lot of redoing, because the way
% the code is written is very very particular to the way memory is stored. 
% 
% "Load-bearing boilerplate"
%
% If you try doing all the MC stuff in a for loop, what you get is 4 nested
% for loops iterating over:
%       - M trajectories
%       - N iterations each
%       - mu_r x mu_l testcases for each iteration
%       - 3 states each (x,y,theta)
%
% and that sucks for speed. even if you parfor the outside, you can get
% everything down to maybe ~1.25 hours. The thing is, we can parfor each step. 
% Batching makes it work, if you had a cache the size of your RAM. the issue is cache misses, 
% and something called memory thrashing. When the array is so big it can't physically fit
% inside your cache, the CPU spends a lot of time 1) searching for the
% data, 2) reorganizing the cache to do all of these math operations. 
%
% But more importantly, MATLAB does addition like this: A = B + C:
%       1) create A the size of B (or C) which can be a million long
%       2) go over each B and C idx, add them together save into A
% So the CPU tries making this faster by putting B and C into cache to do
% the math faster and reduce the "finding data" overhead. But if B and C
% are large, so is A, and they all try to fit in the cache at the same
% time. The CPU ends up taking data from RAM, filling the cache, and then
% sending the data back and taking the next chunk from RAM, which can be a
% lot of roundtrips for a single operation. 
%
% The question is how do we reduce the number of temporary allocations?
% Well, we reorder the way we do operations. If vectorization won't really
% work because the data size is too big, we do things sequentially. 
%
% If we do things sequentially, the order really doesn't matter, we can
% put the simulation on the inside, or on the outside. But considering that
% we have the fun fact that everything should be able to be parfored, with
% the exception of the physics simulation since each step is dependent on
% the previous state, we need to order things differently to take advantage
% of parfor. 
%
% Since everything inside a parfor has to be parallizable, we have to put
% the physics step on the outside, since it can't be parallizable; it has
% to be sequential since the previous state's data is necessary for the
% next state. 
%
% Next, we say that each trajectory gets a processor. At this point, it
% doesn't really matter which for loop goes on the outside. Putting the
% trajectory on the outside makes a lot of sense since it has the fewest
% number of iterations to complete for the most part, and since intuitively
% it makes sense to have each trajectory compute it's own testcases. 
%
% Now for each trajectory, we have to compute 1) the slip ratio/efficiency
% test cases, and 2) do that for every iteration. The order of this doesn't
% really matter. Either way we stride across data to get the next chunk of
% memory. 
% 
% Since we want to operations on the current state of all the different
% testcases, we put that in column major so we can load that into cache as
% a contiguous block. 
%
function [mega_max_spatial, mega_max_theta, sum_x, sum_y, sum_x2, sum_y2] = smc_core_loop(...
    K, eps_v, r, B, max_w, dt, decim, noise_scales, ...
    mu_R_mega, mu_L_mega, ...
    t_X_all, t_Y_all, t_Th_all, t_Xd_all, t_Yd_all, t_Thd_all, ...
    M, F, N, W, n_steps) %#codegen
    
    % N iterations of F testcases gives me this many runs per trajectory
    P        = F * N;

    % Body Geometry
    inv_r    = 1 / r;
    r_over_2 = r / 2;
    r_over_B = r / B;
    half_B   = B / 2;
    two_pi   = single(2 * pi);

    % Controller data
    K_1 = K(1);  
    K_2 = K(2);  
    K_3 = K(3);
    inv_2_epsv = 1 / (2 * eps_v);

    % Noise data
    ns_1 = noise_scales(1)^2;
    ns_2 = noise_scales(2)^2;
    ns_3 = noise_scales(3)^2;

    % State data
    x_curr = zeros(3, P, M, 'single');
    u_curr = zeros(2, P, M, 'single');

    % This doesn't matter as long as it aligns with the state data
    % representation so we can query with the same idx
    mu_R_2D = reshape(mu_R_mega, P, M);
    mu_L_2D = reshape(mu_L_mega, P, M);

    % this is where we load the data into column major form, storing all
    % the states for a trajectory and all its testcases into one contiguous
    % array in memory
    for m = 1:M
        th = t_Th_all(m, 1);
        while th >= two_pi; th = th - two_pi; end
        while th < 0;       th = th + two_pi; end

        x_curr(1, :, m) = t_X_all(m, 1);
        x_curr(2, :, m) = t_Y_all(m, 1);
        x_curr(3, :, m) = th;
    end

    % Accumulators
    mega_max_spatial_3D = zeros(P, M, 'single');
    mega_max_theta_3D   = zeros(P, M, 'single');
    sum_x  = zeros(M, F, n_steps, 'single');
    sum_y  = zeros(M, F, n_steps, 'single');
    sum_x2 = zeros(M, F, n_steps, 'single');
    sum_y2 = zeros(M, F, n_steps, 'single');

    % Time loop
    step_timer = tic;
    for i = 1:n_steps
        update_ctrl = (mod(i - 1, decim) == 0);
        
        % Trajectory loop, do this parallel
        parfor m = 1:M

            % desired trajectory
            ref_X  = t_X_all(m, i);
            ref_Y  = t_Y_all(m, i);
            ref_Th = t_Th_all(m, i);

            % C-safe wrap 2-pi that's very fast
            while ref_Th >= two_pi; ref_Th = ref_Th - two_pi; end
            while ref_Th < 0;       ref_Th = ref_Th + two_pi; end

            % Get the desired derivatives
            ref_Xd = single(0);  ref_Yd = single(0);  ref_Thd = single(0);
            if update_ctrl
                ref_Xd  = t_Xd_all(m, i);
                ref_Yd  = t_Yd_all(m, i);
                ref_Thd = t_Thd_all(m, i);
            end

            % copy the slices to do math on them, and reconcile after the
            % process is done (m is the trajectory!)
            loc_x  = x_curr(:, :, m);
            loc_u  = u_curr(:, :, m);
            loc_mR = mu_R_2D(:, m);
            loc_mL = mu_L_2D(:, m);
            loc_ms = mega_max_spatial_3D(:, m);
            loc_mt = mega_max_theta_3D(:, m);

            loc_sx  = zeros(1, F, 'single');
            loc_sy  = zeros(1, F, 'single');
            loc_sx2 = zeros(1, F, 'single');
            loc_sy2 = zeros(1, F, 'single');
            loc_noise = randn(3, P, 'single');

            % slip ratio testcase
            for f = 1:F
                p_base = (f - 1) * N;
                % run iterations
                for n = 1:N
                    p = p_base + n;

                    c_th = cos(loc_x(3, p));
                    s_th = sin(loc_x(3, p));

                    % Control update. all this stuff is just the same stuff
                    % i've been saying, but unrolled so that C is happy
                    % doing sequential operations. No vectorization here.
                    % just math.
                    if update_ctrl
                        s_1 = loc_x(1, p) - ref_X;
                        s_2 = loc_x(2, p) - ref_Y;

                        s_3 = loc_x(3, p) - ref_Th;
                        while s_3 > single(pi);  s_3 = s_3 - two_pi; end
                        while s_3 < single(-pi); s_3 = s_3 + two_pi; end

                        v_nom = r_over_2 * (loc_u(1, p) + loc_u(2, p));
                        w_nom = r_over_B * (loc_u(1, p) - loc_u(2, p));

                        L_1 = -K_1 * tanh(s_1 * inv_2_epsv) - v_nom * c_th + ref_Xd;
                        L_2 = -K_2 * tanh(s_2 * inv_2_epsv) - v_nom * s_th + ref_Yd;
                        L_3 = -K_3 * tanh(s_3 * inv_2_epsv) - w_nom + ref_Thd;

                        L_proj = L_1 * c_th + L_2 * s_th;

                        % this is the clipping function, just made... into
                        % this...
                        loc_u(1, p) = max(min(loc_u(1, p) + (L_proj + L_3 * half_B) * inv_r, max_w), -max_w);
                        loc_u(2, p) = max(min(loc_u(2, p) + (L_proj - L_3 * half_B) * inv_r, max_w), -max_w);
                    end

                    % Apply the control
                    uR = loc_mR(p) * loc_u(1, p);
                    uL = loc_mL(p) * loc_u(2, p);

                    x1 = loc_x(1, p) + (r_over_2 * (uR + uL) * c_th + ns_1 * loc_noise(1, p)) * dt;
                    x2 = loc_x(2, p) + (r_over_2 * (uR + uL) * s_th + ns_2 * loc_noise(2, p)) * dt;

                    x3 = loc_x(3, p) + (r_over_B * (uR - uL) + ns_3 * loc_noise(3, p)) * dt;
                    while x3 >= two_pi; x3 = x3 - two_pi; end
                    while x3 < 0;       x3 = x3 + two_pi; end

                    loc_x(1, p) = x1;
                    loc_x(2, p) = x2;
                    loc_x(3, p) = x3;

                    % statistics calcs
                    % envelope calculation
                    e1 = x1 - ref_X;
                    e2 = x2 - ref_Y;
                    sp = e1 * e1 + e2 * e2;
                    if sp > loc_ms(p) * loc_ms(p)
                        loc_ms(p) = sqrt(sp);
                    end
                    
                    e3 = x3 - ref_Th;
                    while e3 > single(pi);  e3 = e3 - two_pi; end
                    while e3 < single(-pi); e3 = e3 + two_pi; end
                    ae3 = abs(e3);
                    if ae3 > loc_mt(p)
                        loc_mt(p) = ae3;
                    end

                    % stoer the stats
                    loc_sx(f)  = loc_sx(f)  + x1;
                    loc_sy(f)  = loc_sy(f)  + x2;
                    loc_sx2(f) = loc_sx2(f) + x1 * x1;
                    loc_sy2(f) = loc_sy2(f) + x2 * x2;
                end
            end

            % copy the data back
            x_curr(:, :, m)          = loc_x;
            u_curr(:, :, m)          = loc_u;
            mega_max_spatial_3D(:,m) = loc_ms;
            mega_max_theta_3D(:,m)   = loc_mt;
            sum_x(m, :, i)  = loc_sx;
            sum_y(m, :, i)  = loc_sy;
            sum_x2(m, :, i) = loc_sx2;
            sum_y2(m, :, i) = loc_sy2;
        end

        if mod(i, 1000) == 0
            fprintf("step %.1f / %.1f  (%.2f s per 1k)\n", i, n_steps, toc(step_timer));
            step_timer = tic;
        end
    end

    mega_max_spatial = reshape(mega_max_spatial_3D, 1, W);
    mega_max_theta   = reshape(mega_max_theta_3D, 1, W);
end
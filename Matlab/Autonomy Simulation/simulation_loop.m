function simulation_loop(testcase, dynamics_function, geom, opts)
    %% Define default values for input arguments
    arguments
        testcase = Testcase(0, 1000, 1000, 0:15:100, 0.01, 0.02) %testcase object with trajectory
        dynamics_function = @FullStateDynamics %dynamics function with '_mex' C codegen
        geom = struct('r', 0.17, 'B', 0.2, 'L', 0.25); %geometry parameters for dynamics function
        opts.f_control = 20 %control loop frequency, Hz
        opts.f_estimation = 100 %estimation loop frequency, Hz
        opts.plot_info = struct('toPlot', false, 'steps_per_frame', 1)
    end
    dt = mean(diff(testcase.traj.t_master));
    dynamics_function = strcat(func2str(dynamics_function), "_mex");
    dynamics_function = str2func(dynamics_function);
    %% extract trajectory and components
    Traj = testcase.traj;
    n_steps = length(testcase.traj.t_master);
    t_X  = Traj.X;       t_Y  = Traj.Y;       t_Th  = Traj.Theta;
    t_Xd = Traj.X_dot;   t_Yd = Traj.Y_dot;   t_Thd = Traj.Theta_dot;

    %% Buffers
    x_hist = zeros(3, n_steps);
    u_hist = zeros(4, n_steps);
    s_hist = zeros(3, n_steps);

    
    %% initial state
    full_state = [
    t_X(1);                 % X
    t_Y(1);                 % Y
    testcase.Z_traj(1);     % Z
    0;                      % Vx
    0;                      % Vy
    0;                      % Vz
    0;                      % Roll
    0;                      % Pitch
    mod(t_Th(1), 2 * pi);   % Yaw (initial heading)
    ];
    u = [0; 0; 0; 0];


    %why are there no ymin, dy in testcase?
    dynamics_function(full_state, u, testcase.xmin, testcase.xmin, testcase.dx, testcase.dx, testcase.Z_map, geom, dt, []);
    



    for i = 1:n_steps
        for sub = 1:opts.steps_per_frame
            mission_time = mission_time + dt;
            dstate = dynamics_function(full_state, u, testcase.xmin, testcase.xmin, testcase.dx, testcase.dx, testcase.Z_map, geom, dt, []);
            
        end
    end
end